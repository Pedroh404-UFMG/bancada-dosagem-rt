PROGRAM PLC_PRG
VAR
    // =================================================================
    // 1. ARRAYS DE MAPEAMENTO MODBUS (Vincular na linha raiz de cada canal)
    // =================================================================
    // No Modbus_Server_COM_Port (Serial RS-485):
    Leitura_RTU : ARRAY [0..5] OF WORD; // Vincular na raiz 'Leitura' (%IW) do RTU
    Escrita_RTU : ARRAY [0..6] OF WORD; // Vincular na raiz 'Escrita' (%QW) do RTU

    // No Modbus_TCP_Server (Wi-Fi):
    Leitura_TCP : ARRAY [0..5] OF WORD; // Vincular na raiz 'Leitura' (%IW) do TCP
    Escrita_TCP : ARRAY [0..6] OF WORD; // Vincular na raiz 'Escrita' (%QW) do TCP

    // Variáveis Espelhadas Ativas (Unificadas pela chave seletora)
    PesoSilo         : WORD; // Offset 0 - Decigramas (0,1 g) - Silo Rápido
    PesoCaixaA       : WORD; // Offset 1 - Decigramas (0,1 g) - Caixa A Filtrada
    PesoCaixaB       : WORD; // Offset 2 - Decigramas (0,1 g) - Caixa B Filtrada
    RegReservado     : WORD; // Offset 3 - Reservado
    RegStatusChaves  : WORD; // Offset 4 - Bit 0: FC_CaixaA | Bit 1: FC_CaixaB
    WdESP32          : WORD; // Offset 5 - Watchdog do Escravo (0 a 1000)

    CmdValvula       : WORD; // Offset 10 - 0 a 100% de Abertura
    CmdTracaoEsteira : WORD; // Offset 11 - 0 a 255 PWM (Saída após rampa)
    CmdGiroN20_PWM   : WORD; // Offset 12 - 0 a 255 PWM (Saída após rampa)
    CmdGiroN20_Dir   : WORD; // Offset 13 - 0 (Caixa A) ou 1 (Caixa B)
    CmdTara          : WORD; // Offset 14 - Bitmask Tara (1=Silo, 2=CxA, 4=CxB)
    WdCodesys        : WORD; // Offset 15 - Watchdog do Mestre (0 a 1000)
    CmdModoRede      : WORD; // Offset 16 - 0 = Serial RTU / 1 = Wi-Fi TCP

    // =================================================================
    // 2. VARIÁVEIS DE DOSAGEM LOSS-IN-WEIGHT E COMPENSAÇÃO (PRE-ACT)
    // =================================================================
    SetpointPeso_dg    : INT := 500; // Meta inicial: 50,0 g (faixa 12,7g a 82,7g)
    PreAct_dg          : INT := 15;  // Material em voo inicial: 1,5 g (15 dg)
    PesoSiloInicial_dg : INT;         // Retrato do silo antes de abrir a válvula
    PontoDeCorte_dg    : INT;         // Alvo de perda de massa para fechar a válvula
    PesoDestinoIni_dg  : INT;         // Retrato da caixa antes do ciclo
    PesoDosado_dg      : INT;         // Massa que saiu do silo em tempo real
    PesoRecebido_dg    : INT;         // Massa efetivamente recebida na caixa
    ErroDosagem_dg     : INT;         // Erro da batelada em decigramas

    // Variáveis convertidas para GRAMAS REAIS (Exibição no WebVisu)
    PesoSilo_g         : REAL;
    PesoCaixaA_g       : REAL;
    PesoCaixaB_g       : REAL;
    SetpointPeso_g     : REAL;
    PreAct_g           : REAL;
    ErroDosagem_g      : REAL;

    // =================================================================
    // 3. VARIÁVEIS INTERNAS, RAMPA PWM E SUPERVISÓRIO (IHM)
    // =================================================================
    FC_CaixaA          : BOOL;
    FC_CaixaB          : BOOL;
    VS_IniciarDosagemA : BOOL; // Botão na IHM: Dosar na Caixa A
    VS_IniciarDosagemB : BOOL; // Botão na IHM: Dosar na Caixa B
    VS_TararSilo       : BOOL; // Botão na IHM: Tarar Silo
    VS_TararCaixas     : BOOL; // Botão na IHM: Tarar Caixas A e B
    VS_Modo_WiFi       : BOOL; // Chave seletora na IHM (FALSE = RTU, TRUE = Wi-Fi)

    eEstado            : (PARADO, BUSCANDO_POSICAO, CARREGANDO, LIMPANDO, REVERTENDO);
    CaixaAlvo          : INT;
    TMR_Limpeza        : TON;
    EstadoTexto        : STRING;
    LampadaValvula     : BOOL;

    // Controle de Rampa PWM (Soft-Start)
    AlvoTracaoEsteira  : WORD := 0;
    AlvoGiroN20        : WORD := 0;
    PassoRampaPWM      : WORD := 4; 

    // Simulação do Gauge da Esteira (0.0 = Caixa A, 100.0 = Caixa B)
    PosicaoEsteiraSimulada : REAL := 50.0;

    // Monitoramento de Watchdog
    WdESP32_Anterior   : WORD;
    TMR_FalhaRede      : TON;
    AlarmeFalhaRede    : BOOL;

    // =================================================================
    // 4. VARIÁVEIS DO TRACE (Prova de Conceito TCC)
    // =================================================================
    hTask              : CmpIecTask.RTS_IEC_HANDLE;
    Result             : CmpIecTask.RTS_IEC_RESULT;
    pCurrentTaskInfo   : POINTER TO CmpIecTask.Task_Info2;
    TempoCicloAtual    : UDINT; 
    TempoCicloMax      : UDINT;
    JitterAtual        : DINT;
    JitterMax          : DINT;
END_VAR


// =====================================================================
// INÍCIO DA LÓGICA
// =====================================================================

// 1. SELEÇÃO DE REDE E MULTIPLEXAÇÃO DE LEITURA (TCP vs. RTU)
IF VS_Modo_WiFi THEN
    CmdModoRede     := 1;
    PesoSilo        := Leitura_TCP[0];
    PesoCaixaA      := Leitura_TCP[1];
    PesoCaixaB      := Leitura_TCP[2];
    RegReservado    := Leitura_TCP[3];
    RegStatusChaves := Leitura_TCP[4];
    WdESP32         := Leitura_TCP[5];
ELSE
    CmdModoRede     := 0;
    PesoSilo        := Leitura_RTU[0];
    PesoCaixaA      := Leitura_RTU[1];
    PesoCaixaB      := Leitura_RTU[2];
    RegReservado    := Leitura_RTU[3];
    RegStatusChaves := Leitura_RTU[4];
    WdESP32         := Leitura_RTU[5];
END_IF;

// 2. WATCHDOG DO MESTRE E DETECÇÃO VISUAL DE FALHA DE REDE
WdCodesys := (WdCodesys + 1) MOD 1000;

TMR_FalhaRede(IN := (WdESP32 = WdESP32_Anterior), PT := T#600MS);
WdESP32_Anterior := WdESP32;
AlarmeFalhaRede := TMR_FalhaRede.Q;

// 3. CONVERSÃO PARA GRAMAS (EXIBIÇÃO NO SUPERVISÓRIO) E SENSORES
PesoSilo_g     := WORD_TO_REAL(PesoSilo) / 10.0;
PesoCaixaA_g   := WORD_TO_REAL(PesoCaixaA) / 10.0;
PesoCaixaB_g   := WORD_TO_REAL(PesoCaixaB) / 10.0;
SetpointPeso_g := INT_TO_REAL(SetpointPeso_dg) / 10.0;
PreAct_g       := INT_TO_REAL(PreAct_dg) / 10.0;

FC_CaixaA      := RegStatusChaves.0;
FC_CaixaB      := RegStatusChaves.1;
EstadoTexto    := TO_STRING(eEstado);
LampadaValvula := (CmdValvula > 0);

// 4. COMANDOS MANUAIS DE TARA PELA IHM (Apenas quando PARADO)
IF eEstado = PARADO THEN
    IF VS_TararSilo THEN
        CmdTara := 1; // Bit 0 = Silo
        VS_TararSilo := FALSE;
    ELSIF VS_TararCaixas THEN
        CmdTara := 6; // Bits 1 e 2 = Caixa A (2) + Caixa B (4)
        VS_TararCaixas := FALSE;
    ELSE
        CmdTara := 0;
    END_IF;
ELSE
    CmdTara := 0;
END_IF;

// 5. MÁQUINA DE ESTADOS (LOSS-IN-WEIGHT COM CORTE ANTECIPADO)
CASE eEstado OF
    
    PARADO:
        CmdValvula := 0;
        AlvoTracaoEsteira := 0;
        AlvoGiroN20 := 0;
        
        IF VS_IniciarDosagemA THEN
            CaixaAlvo := 1;
            eEstado := BUSCANDO_POSICAO;
            VS_IniciarDosagemA := FALSE;
        ELSIF VS_IniciarDosagemB THEN
            CaixaAlvo := 2;
            eEstado := BUSCANDO_POSICAO;
            VS_IniciarDosagemB := FALSE;
        END_IF;

    BUSCANDO_POSICAO:
        IF CaixaAlvo = 1 THEN
            IF FC_CaixaA THEN
                AlvoGiroN20 := 0;
                PesoSiloInicial_dg := WORD_TO_INT(PesoSilo);
                PesoDestinoIni_dg  := WORD_TO_INT(PesoCaixaA);
                PontoDeCorte_dg    := SetpointPeso_dg - PreAct_dg;
                IF PontoDeCorte_dg < 10 THEN PontoDeCorte_dg := 10; END_IF;
                eEstado := CARREGANDO;
            ELSE
                CmdGiroN20_Dir := 0;
                AlvoGiroN20 := 200;
            END_IF;
            
        ELSIF CaixaAlvo = 2 THEN
            IF FC_CaixaB THEN
                AlvoGiroN20 := 0;
                PesoSiloInicial_dg := WORD_TO_INT(PesoSilo);
                PesoDestinoIni_dg  := WORD_TO_INT(PesoCaixaB);
                PontoDeCorte_dg    := SetpointPeso_dg - PreAct_dg;
                IF PontoDeCorte_dg < 10 THEN PontoDeCorte_dg := 10; END_IF;
                eEstado := CARREGANDO;
            ELSE
                CmdGiroN20_Dir := 1;
                AlvoGiroN20 := 200;
            END_IF;
        END_IF;

    CARREGANDO:
        AlvoTracaoEsteira := 200; 
        CmdValvula := 100;        
        
        // Leitura direta do silo sem atraso de fase (Silo Rápido)
        PesoDosado_dg := PesoSiloInicial_dg - WORD_TO_INT(PesoSilo);
        IF PesoDosado_dg < 0 THEN
            PesoDosado_dg := 0;
        END_IF;
        
        // Corte antecipado testando o determinismo do PREEMPT_RT
        IF PesoDosado_dg >= PontoDeCorte_dg THEN
            CmdValvula := 0; 
            eEstado := LIMPANDO;
        END_IF;

    LIMPANDO:
        AlvoTracaoEsteira := 150; 
        TMR_Limpeza(IN := TRUE, PT := T#4S);
        
        IF TMR_Limpeza.Q THEN
            TMR_Limpeza(IN := FALSE);
            AlvoTracaoEsteira := 0;
            
            // --- COMPENSAÇÃO ADAPTATIVA PÓS-CICLO (PRE-ACT) USANDO CAIXAS FILTRADAS ---
            IF CaixaAlvo = 1 THEN
                PesoRecebido_dg := WORD_TO_INT(PesoCaixaA) - PesoDestinoIni_dg;
            ELSE
                PesoRecebido_dg := WORD_TO_INT(PesoCaixaB) - PesoDestinoIni_dg;
            END_IF;
            
            IF PesoRecebido_dg < 0 THEN PesoRecebido_dg := 0; END_IF;
            
            ErroDosagem_dg := PesoRecebido_dg - SetpointPeso_dg;
            ErroDosagem_g  := INT_TO_REAL(ErroDosagem_dg) / 10.0;
            
            PreAct_dg := PreAct_dg + (ErroDosagem_dg / 2);
            IF PreAct_dg < 0 THEN PreAct_dg := 0; END_IF;
            IF PreAct_dg > (SetpointPeso_dg / 2) THEN PreAct_dg := SetpointPeso_dg / 2; END_IF;
            
            eEstado := REVERTENDO;
        END_IF;

    REVERTENDO:
        AlvoTracaoEsteira := 0; 
        AlvoGiroN20 := 0;
        eEstado := PARADO;

END_CASE;

// 6. EXECUÇÃO DAS RAMPAS DE ACELERAÇÃO PWM (SOFT-START)
IF AlvoTracaoEsteira = 0 THEN
    CmdTracaoEsteira := 0; 
ELSIF CmdTracaoEsteira < AlvoTracaoEsteira THEN
    CmdTracaoEsteira := CmdTracaoEsteira + PassoRampaPWM;
    IF CmdTracaoEsteira > AlvoTracaoEsteira THEN CmdTracaoEsteira := AlvoTracaoEsteira; END_IF;
ELSIF CmdTracaoEsteira > AlvoTracaoEsteira THEN
    CmdTracaoEsteira := AlvoTracaoEsteira;
END_IF;

IF AlvoGiroN20 = 0 THEN
    CmdGiroN20_PWM := 0; 
ELSIF CmdGiroN20_PWM < AlvoGiroN20 THEN
    CmdGiroN20_PWM := CmdGiroN20_PWM + PassoRampaPWM;
    IF CmdGiroN20_PWM > AlvoGiroN20 THEN CmdGiroN20_PWM := AlvoGiroN20; END_IF;
ELSIF CmdGiroN20_PWM > AlvoGiroN20 THEN
    CmdGiroN20_PWM := AlvoGiroN20;
END_IF;

// 7. SIMULAÇÃO MATEMÁTICA DO GAUGE DE POSIÇÃO DA ESTEIRA
IF FC_CaixaA THEN
    PosicaoEsteiraSimulada := 0.0;
ELSIF FC_CaixaB THEN
    PosicaoEsteiraSimulada := 100.0;
ELSIF CmdGiroN20_PWM > 0 THEN
    IF CmdGiroN20_Dir = 0 THEN
        PosicaoEsteiraSimulada := PosicaoEsteiraSimulada - 0.5;
        IF PosicaoEsteiraSimulada < 0.0 THEN PosicaoEsteiraSimulada := 0.0; END_IF;
    ELSE
        PosicaoEsteiraSimulada := PosicaoEsteiraSimulada + 0.5;
        IF PosicaoEsteiraSimulada > 100.0 THEN PosicaoEsteiraSimulada := 100.0; END_IF;
    END_IF;
END_IF;

// 8. ATUALIZAÇÃO DOS BLOCOS DE ESCRITA MODBUS (AMBAS AS REDES)
Escrita_RTU[0] := CmdValvula;       Escrita_TCP[0] := CmdValvula;
Escrita_RTU[1] := CmdTracaoEsteira; Escrita_TCP[1] := CmdTracaoEsteira;
Escrita_RTU[2] := CmdGiroN20_PWM;   Escrita_TCP[2] := CmdGiroN20_PWM;
Escrita_RTU[3] := CmdGiroN20_Dir;   Escrita_TCP[3] := CmdGiroN20_Dir;
Escrita_RTU[4] := CmdTara;          Escrita_TCP[4] := CmdTara;
Escrita_RTU[5] := WdCodesys;        Escrita_TCP[5] := WdCodesys;
Escrita_RTU[6] := CmdModoRede;      Escrita_TCP[6] := CmdModoRede;

// 9. AQUISIÇÃO DE DADOS (TRACE) - INTOCADO
hTask := CmpIecTask.IecTaskGetCurrent(ADR(Result));
IF hTask <> 0 THEN
    pCurrentTaskInfo := CmpIecTask.IecTaskGetInfo3(hTask, ADR(Result));
    IF pCurrentTaskInfo <> 0 THEN
        TempoCicloAtual := pCurrentTaskInfo^.dwCycleTime; 
        TempoCicloMax   := pCurrentTaskInfo^.dwMaxCycleTime;
        JitterAtual     := pCurrentTaskInfo^.iJitter;      
        JitterMax       := pCurrentTaskInfo^.iJitterMax;   
    END_IF
END_IF;
