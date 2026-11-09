#!/usr/bin/env python3
import os
import glob
import time
import termios
import threading
import subprocess
from gpiozero import Button, LED

# ==========================================
# 1. CONSTANTES E DETECÇÃO AUTOMÁTICA DO KERNEL
# ==========================================
TEMPO_PRESSAO_LONGA = 3.0
TEMPO_FEEDBACK = 2.0
botao_foi_segurado = False
acao_em_andamento = False
processo_stress = None
stress_ativo = False
thread_ataque = None

info_uname = os.uname()
MODO_KERNEL_RT = ('rt' in info_uname.release.lower()) or ('preempt_rt' in info_uname.version.lower())

# ==========================================
# 2. DEFINIÇÃO DE HARDWARE (PINOS COM DEBOUNCE)
# ==========================================
botao_reboot = Button(27, pull_up=True, bounce_time=0.08, hold_time=TEMPO_PRESSAO_LONGA)
led_ssh = LED(24)

chave_stress = Button(25, pull_up=True, bounce_time=0.10)
led_stress = LED(26)

# ==========================================
# 3. FUNÇÕES DE REBOOT E SHUTDOWN (Energia)
# ==========================================
def acao_segurar():
    global botao_foi_segurado, acao_em_andamento
    if acao_em_andamento:
        return
    botao_foi_segurado = True
    acao_em_andamento = True
    led_ssh.blink(on_time=0.5, off_time=0.5)
    time.sleep(TEMPO_FEEDBACK)
    limpar_rede_wifi()
    os.system("sudo shutdown -h now")

def acao_soltar():
    global botao_foi_segurado, acao_em_andamento
    if acao_em_andamento:
        return
    if not botao_foi_segurado:
        acao_em_andamento = True
        led_ssh.blink(on_time=0.1, off_time=0.1)
        time.sleep(TEMPO_FEEDBACK)
        limpar_rede_wifi()
        os.system("sudo reboot")
    botao_foi_segurado = False

# ==========================================
# 4. CONTROLE DE TRÁFEGO WIFI (PORTA 502) E SERIAL (ttyUSB0)
#    NÃO usa SIGSTOP -> NUNCA causa EXCEÇÃO no CODESYS!
#    NÃO afeta porta 8080 (WebVisu) nem porta 22 (SSH)!
# ==========================================
def inicializar_fila_wifi_porta_502():
    """Cria a árvore prio na wlan0 onde:
    - Banda 1:1 = Via expressa (0ms) para WebVisu (8080) e SSH (22).
    - Banda 1:3 = Exclusiva para Modbus TCP (porta 502)."""
    os.system("sudo tc qdisc del dev wlan0 root 2>/dev/null")
    regras = [
        "sudo tc qdisc add dev wlan0 root handle 1: prio bands 3 priomap 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0",
        "sudo tc qdisc add dev wlan0 parent 1:3 handle 30: netem delay 0ms",
        "sudo tc filter add dev wlan0 protocol ip parent 1:0 prio 3 u32 match ip dport 502 0xffff flowid 1:3",
        "sudo tc filter add dev wlan0 protocol ip parent 1:0 prio 3 u32 match ip sport 502 0xffff flowid 1:3"
    ]
    for r in regras:
        os.system(r + " 2>/dev/null")

def alternar_bloqueio_wifi_502(bloquear=True):
    """Altera apenas a banda 1:3 (porta 502) sem tocar no WebVisu nem no SSH."""
    if bloquear:
        os.system("sudo tc qdisc change dev wlan0 parent 1:3 handle 30: netem delay 3000ms 500ms loss 95% 2>/dev/null")
    else:
        os.system("sudo tc qdisc change dev wlan0 parent 1:3 handle 30: netem delay 0ms loss 0% 2>/dev/null")

def limpar_rede_wifi():
    os.system("sudo tc qdisc del dev wlan0 root 2>/dev/null")

def obter_tids_codesys():
    tids = []
    try:
        out = subprocess.check_output(['pgrep', '-f', 'codesyscontrol'], text=True).strip().split()
        for pid in out:
            for comm_path in glob.glob(f'/proc/{pid}/task/[0-9]*/comm'):
                with open(comm_path, 'r') as f:
                    nome = f.read().strip().lower()
                if ('maintask' in nome) or ('plctask' in nome) or ('iec' in nome) or ('comm' in nome):
                    tids.append(comm_path.split('/')[-2])
    except Exception:
        pass
    return tids

def loop_estresse_kernel_normal():
    """Executado APENAS no Kernel Normal (Desktop).
    Sem pkill -STOP (zero risco de EXCEÇÃO!).
    Usa janela de 0.35s aberta e 7.50s retida -> faz o silo despejar 80g a 110g
    e a esteira transbordar a caixa (> +30g de erro) tanto em Serial quanto em Wi-Fi!"""
    global stress_ativo
    fd_serial = None
    try:
        if os.path.exists('/dev/ttyUSB0'):
            fd_serial = os.open('/dev/ttyUSB0', os.O_RDWR | os.O_NONBLOCK | os.O_NOCTTY)
    except Exception:
        fd_serial = None

    try:
        inicializar_fila_wifi_porta_502()

        # Coloca as tasks de controle nos núcleos 2 e 3 junto com o stress-ng (gera Jitter seguro sem EXCEÇÃO)
        for tid in obter_tids_codesys():
            os.system(f"sudo taskset -cp 2,3 {tid} >/dev/null 2>&1")

        while stress_ativo:
            # 1) Janela curta de 0.35s ABERTA:
            #    Permite que o clique de "Iniciar Dosagem" posicione a esteira e abra a válvula em 100%,
            #    mas fecha antes de atingir os 12g (impedindo a válvula de fechar parcial ou cortar!)
            alternar_bloqueio_wifi_502(False)
            time.sleep(0.35)
            if not stress_ativo:
                break

            # 2) Janela longa de 7.50s DE FALHA CONTÍNUA (Wi-Fi e Serial RTU):
            #    Mantém a válvula aberta e a esteira rodando por 7.5s inteiros -> Erro > +30g a +50g!
            alternar_bloqueio_wifi_502(True)
            t_fim = time.time() + 7.50
            while stress_ativo and (time.time() < t_fim):
                if fd_serial is not None:
                    try:
                        termios.tcflush(fd_serial, termios.TCIFLUSH)
                        os.read(fd_serial, 1024)
                    except Exception:
                        pass
                time.sleep(0.002) # 2ms: não deixa escapar nenhum pacote na Serial e gasta < 2% de CPU
    finally:
        limpar_rede_wifi()
        if fd_serial is not None:
            try:
                os.close(fd_serial)
            except Exception:
                pass
        for tid in obter_tids_codesys():
            os.system(f"sudo taskset -cp 0-3 {tid} >/dev/null 2>&1")

# ==========================================
# 5. ACIONAMENTO DA CHAVE DE ESTRESSE
# ==========================================
def ligar_stress():
    global processo_stress, stress_ativo, thread_ataque
    if not stress_ativo:
        stress_ativo = True
        subprocess.run(['sudo', 'pkill', '-9', 'stress-ng'], stderr=subprocess.DEVNULL)
        led_stress.blink(on_time=0.2, off_time=0.2)

        # Carga de CPU/Context Switch nos núcleos 2 e 3 (Núcleos 0 e 1 livres p/ SSH e WebVisu)
        # Sem chrt -f 95 e sem SIGSTOP -> NUNCA derruba o CODESYS em EXCEÇÃO!
        cmd = [
            'stress-ng',
            '--taskset', '2,3',
            '--cpu', '2',
            '--cpu-load', '90',
            '--switch', '4',
            '--timer', '2',
            '--vm', '1',
            '--vm-bytes', '64M'
        ]
        processo_stress = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        if not MODO_KERNEL_RT:
            thread_ataque = threading.Thread(target=loop_estresse_kernel_normal, daemon=True)
            thread_ataque.start()

def desligar_stress():
    global processo_stress, stress_ativo
    stress_ativo = False
    limpar_rede_wifi()
    if processo_stress is not None:
        processo_stress.terminate()
        processo_stress = None
    subprocess.run(['sudo', 'pkill', '-9', 'stress-ng'], stderr=subprocess.DEVNULL)
    led_stress.off()

# ==========================================
# 6. ATRIBUIÇÃO DE EVENTOS E LOOP PRINCIPAL
# ==========================================
botao_reboot.when_held = acao_segurar
botao_reboot.when_released = acao_soltar

chave_stress.when_pressed = ligar_stress
chave_stress.when_released = desligar_stress

limpar_rede_wifi()
led_ssh.on()

if chave_stress.is_pressed:
    ligar_stress()
else:
    led_stress.off()

try:
    while True:
        time.sleep(3600)
except KeyboardInterrupt:
    desligar_stress()
