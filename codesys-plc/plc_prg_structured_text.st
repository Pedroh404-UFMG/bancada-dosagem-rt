PROGRAM PLC_PRG
VAR
    // =================================================================
    // 1. ARRAYS DE MAPEAMENTO MODBUS (Vincular na linha raiz de cada canal)
    // =================================================================
    Leitura_RTU : ARRAY [0..5] OF WORD; // Vincular na raiz 'Leitura' (%IW) do RTU
    Escrita_RTU : ARRAY [0..6] OF WORD; // Vincular na raiz 'Escrita' (%QW) do RTU

    Leitura_TCP : ARRAY [0..5] OF WORD; // Vincular na raiz 'Leitura' (%IW) do TCP
    Escrita_TCP : ARRAY [0..6] OF WORD; // Vincular na raiz 'Escrita' (%QW) do TCP

    // Variáveis Espelhadas Ativas
    PesoSiloBruto    : WORD; // Offset 0 - Leitura bruta vinda do ESP32 (dg)
    PesoSilo         : INT;  // Peso líquido do Silo já descontando a canaleta (dg)
    PesoCaixaA       : WORD; // Offset 1 - Decigramas (0,1 g)
    PesoCaixaB       : WORD; // Offset 2 - Decigramas (0,1 g)
    RegReservado     : WORD; // Offset 3 - Reservado
    RegStatusChaves  : WORD; // Offset 4 - Bits das Chaves Fim de Curso vindos do ESP32
    WdESP32          : WORD; // Offset 5 - Watchdog do Escravo (0 a 1000)

    CmdValvula       : WORD; // Offset 10 - 0 a 100% de Abertura
    CmdTracaoEsteira : WORD; // Offset 11 - 0 a 255 PWM (Cinta)
    CmdGiroTT_PWM    : WORD; // Offset 12 - 0 a 255 PWM (Giro)
    CmdGiroTT_Dir    : WORD; // Offset 13 - Direção do Giro enviada ao ESP32
    CmdTara          : WORD; // Offset 14 - Bitmask Tara no ESP32 (1=Silo, 2=CxA, 4=CxB)
    WdCodesys        : WORD; // Offset 15 - Watchdog do Mestre (0 a 1000)
    CmdModoRede      : WORD; // Offset 16 - 0 = Serial RTU / 1 = Wi-Fi TCP

    // =================================================================
    // 2. SENSOR DE PRESENÇA, TARA LOCAL DO SILO/CANALETA E PESO LÍQUIDO
    // =================================================================
    TaraSiloLocal_dg   : INT := 0;   // Desconta instantaneamente os 30g da canaleta!
    TaraCacambaA_dg    : INT := 126; // Peso padrão da caçamba A vazia (12,6 g)
    TaraCacambaB_dg    : INT := 117; // Peso padrão da caçamba B vazia (11,7 g)
    LimiarPresenca_dg  : INT := 90;  // Acima de 9,0 g (90 dg) -> Acende LED Presença!
    
    PresencaCaixaA     : BOOL;       // LED: Caçamba A presente na base
    PresencaCaixaB     : BOOL;       // LED: Caçamba B presente na base
    SiloComMaterial    : BOOL;       // Trava: Silo possui material suficiente
    
    PesoLiquidoCxA_dg  : INT;        // Peso líquido instantâneo na Caixa A (em dg)
    PesoLiquidoCxB_dg  : INT;        // Peso líquido instantâneo na Caixa B (em dg)

    // =================================================================
    // 3. VARIÁVEIS DE DOSAGEM EM CASCATA E CONTROLE DE VAZÃO DO SILO
    // =================================================================
    SetpointPeso_dg       : INT := 500;  // Meta final na caixa: 50,0 g (500 dg)
    PreAct_dg             : INT := 45;   // Antecipação de fechamento da válvula: 4,5 g (45 dg)
    DoseAberturaMax_dg    : INT := 120;  // Abre 100% nos primeiros 12,0 g e já passa para vazão controlada
    AberturaValvulaMax    : WORD := 100; // Abertura inicial (100%)
    AberturaValvulaFina   : WORD := 68;  // Abertura parcial de controle de vazão (68%)
    
    MargemDesacelera_dg   : INT := 100;  // Faltando 10,0 g na caixa, desacelera a cinta da esteira
    OffsetQuedaEsteira_dg : INT := 0;    // Ajuste de borda da esteira ao parar a cinta
    
    PesoSiloInicial_dg    : INT;         // Retrato do silo antes de abrir a válvula
    AlvoDescargaSilo_dg   : INT;         // Quanto o silo precisa despejar neste ciclo (Setpoint - Estoque)
    PontoDeCorte_dg       : INT;         // Alvo de perda de massa para fechar totalmente a válvula
    PesoDestinoIni_dg     : INT;         // Retrato líquido da caixa antes do ciclo
    PesoDosado_dg         : INT;         // Massa que saiu do silo
    PesoRecebido_dg       : INT;         // Massa líquida efetivamente recebida na caixa
    EstoqueEsteira_dg     : INT := 0;    // Massa retida sobre a esteira para o próximo ciclo (dg)
    VooRealSilo_dg        : INT := 0;    // Massa real que caiu após o comando de fechar a válvula
    ErroDosagem_dg        : INT;         // Erro da batelada em decigramas

    // Variáveis convertidas para GRAMAS REAIS (Exibição no WebVisu)
    PesoSilo_g            : REAL;
    PesoCaixaA_g          : REAL;
    PesoCaixaB_g          : REAL;
    SetpointPeso_g        : REAL;
    PreAct_g              : REAL;
    ErroDosagem_g         : REAL;
    EstoqueEsteira_g      : REAL;

    // =================================================================
    // 4. VARIÁVEIS INTERNAS, PURGA + ESVAZIAR SILO, RAMPA PWM E IHM
    // =================================================================
    FC_CaixaA           : BOOL;
    FC_CaixaB           : BOOL;
    VS_IniciarDosagemA  : BOOL; // Botão na IHM: Dosar na Caixa A
    VS_IniciarDosagemB  : BOOL; // Botão na IHM: Dosar na Caixa B
    VS_Purga            : BOOL; // Botão na IHM: Vai p/ Caixa A e limpa a esteira por 10s
    VS_EsvaziarSilo     : BOOL; // Botão na IHM: Esvazia o Silo inteiro + Purga a esteira (Limpeza Completa!)
    VS_ParadaEmergencia : BOOL; // Botão na IHM: Corta motores/válvula e volta p/ PARADO
    VS_TararSilo        : BOOL; // Botão na IHM: Tarar Silo (Zera os 30g da canaleta!)
    VS_TararCaixas      : BOOL; // Botão na IHM: Tarar Caixas (Zera caçamba vazia ou base)
    VS_Modo_WiFi        : BOOL; // Chave seletora na IHM (FALSE = RTU, TRUE = Wi-Fi)

    eEstado             : (PARADO, BUSCANDO_POSICAO, CARREGANDO, LIMPANDO, ESTABILIZANDO, REVERTENDO, PURGA_POSICIONANDO, PURGA_ESVAZIANDO_SILO, PURGA_EXECUTANDO);
    CaixaAlvo           : INT;
    FlagLimpezaCompleta : BOOL := FALSE;
    
    // Temporizadores e Tempos Configuráveis
    TempoLimpeza        : TIME := T#10S;     // Timeout máximo de transporte da esteira caso falte material
    TempoEstabilizacao  : TIME := T#2000MS;  // Tempo parado esperando a balança estabilizar (2,0 s)
    TempoPurga          : TIME := T#10S;     // Tempo de limpeza final da esteira
    TempoMaxEsvaziarSilo: TIME := T#25S;     // Timeout máximo de segurança ao esvaziar o silo
    TMR_Limpeza         : TON;
    TMR_Estabilizacao   : TON;
    TMR_Purga           : TON;
    TMR_EsvaziarSilo    : TON;
    TMR_SiloVazio       : TON;
    TMR_TimeoutCarga    : TON;
    TMR_PulsoTara       : TON;
    BufferCmdTara       : WORD := 0;
    TimeoutTransporte   : BOOL := FALSE;

    EstadoTexto         : STRING;
    LampadaValvula      : BOOL;

    // Valores Calibrados de PWM e Rampas Fortes
    PWM_Cinta_Carga     : WORD := 245; // Força máxima para vencer atrito
    PWM_Cinta_Limpeza   : WORD := 195; // Torque firme no ajuste fino
    PWM_Giro_CaixaA     : WORD := 150; // PWM do Giro indo para Caixa A
    PWM_Giro_CaixaB     : WORD := 140; // PWM do Giro indo para Caixa B
    AlvoTracaoEsteira   : WORD := 0;
    AlvoGiroTT          : WORD := 0;
    PassoRampaPWM       : WORD := 25;  // Partida rápida da cinta sem arrastar

    // Simulação do Gauge da Esteira (0.0 = Caixa A, 100.0 = Caixa B)
    PosicaoEsteiraSimulada : REAL := 50.0;

    // Monitoramento de Watchdog
    WdESP32_Anterior   : WORD;
    TMR_FalhaRede      : TON;
    AlarmeFalhaRede    : BOOL;

    // =================================================================
    // 5. VARIÁVEIS DO TRACE (Prova de Conceito TCC)
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

// 0. BOTÃO DE PARADA DE EMERGÊNCIA / ABORTAR CICLO PELA IHM
IF VS_ParadaEmergencia THEN
    eEstado := PARADO;
    FlagLimpezaCompleta := FALSE;
    CmdValvula := 0;
    AlvoTracaoEsteira := 0;
    AlvoGiroTT := 0;
    CmdTracaoEsteira := 0;
    CmdGiroTT_PWM := 0;
    VS_ParadaEmergencia := FALSE;
END_IF;

// 1. SELEÇÃO DE REDE E MULTIPLEXAÇÃO DE LEITURA (TCP vs. RTU)
IF VS_Modo_WiFi THEN
    CmdModoRede     := 1;
    PesoSiloBruto   := Leitura_TCP[0];
    PesoCaixaA      := Leitura_TCP[1];
    PesoCaixaB      := Leitura_TCP[2];
    RegReservado    := Leitura_TCP[3];
    RegStatusChaves := Leitura_TCP[4];
    WdESP32         := Leitura_TCP[5];
ELSE
    CmdModoRede     := 0;
    PesoSiloBruto   := Leitura_RTU[0];
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

// 3. CÁLCULO DO PESO LÍQUIDO DO SILO E DAS CAIXAS
PesoSilo := WORD_TO_INT(PesoSiloBruto) - TaraSiloLocal_dg;
IF PesoSilo < 0 THEN
    PesoSilo := 0;
END_IF;

PresencaCaixaA := (WORD_TO_INT(PesoCaixaA) >= LimiarPresenca_dg);
PresencaCaixaB := (WORD_TO_INT(PesoCaixaB) >= LimiarPresenca_dg);

SiloComMaterial := ((PesoSilo + EstoqueEsteira_dg) >= 50);

IF PresencaCaixaA THEN
    PesoLiquidoCxA_dg := WORD_TO_INT(PesoCaixaA) - TaraCacambaA_dg;
    IF PesoLiquidoCxA_dg < 0 THEN PesoLiquidoCxA_dg := 0; END_IF;
ELSE
    PesoLiquidoCxA_dg := WORD_TO_INT(PesoCaixaA);
END_IF;

IF PresencaCaixaB THEN
    PesoLiquidoCxB_dg := WORD_TO_INT(PesoCaixaB) - TaraCacambaB_dg;
    IF PesoLiquidoCxB_dg < 0 THEN PesoLiquidoCxB_dg := 0; END_IF;
ELSE
    PesoLiquidoCxB_dg := WORD_TO_INT(PesoCaixaB);
END_IF;

PesoSilo_g       := INT_TO_REAL(PesoSilo) / 10.0;
PesoCaixaA_g     := INT_TO_REAL(PesoLiquidoCxA_dg) / 10.0;
PesoCaixaB_g     := INT_TO_REAL(PesoLiquidoCxB_dg) / 10.0;
SetpointPeso_g   := INT_TO_REAL(SetpointPeso_dg) / 10.0;
PreAct_g         := INT_TO_REAL(PreAct_dg) / 10.0;
EstoqueEsteira_g := INT_TO_REAL(EstoqueEsteira_dg) / 10.0;

FC_CaixaA      := RegStatusChaves.1;
FC_CaixaB      := RegStatusChaves.0;
LampadaValvula := (CmdValvula > 0);

// 5. COMANDOS MANUAIS DE TARA PELA IHM
IF eEstado = PARADO THEN
    IF VS_TararSilo THEN
        TaraSiloLocal_dg  := WORD_TO_INT(PesoSiloBruto);
        EstoqueEsteira_dg := 0;
        BufferCmdTara     := 1;
        VS_TararSilo      := FALSE;
    ELSIF VS_TararCaixas THEN
        BufferCmdTara := 0;
        IF PresencaCaixaA THEN
            TaraCacambaA_dg := WORD_TO_INT(PesoCaixaA);
        ELSE
            TaraCacambaA_dg := 126;
            BufferCmdTara := BufferCmdTara + 2;
        END_IF;
        
        IF PresencaCaixaB THEN
            TaraCacambaB_dg := WORD_TO_INT(PesoCaixaB);
        ELSE
            TaraCacambaB_dg := 117;
            BufferCmdTara := BufferCmdTara + 4;
        END_IF;
        VS_TararCaixas := FALSE;
    END_IF;
END_IF;

TMR_PulsoTara(IN := (BufferCmdTara > 0), PT := T#250MS);
IF TMR_PulsoTara.Q THEN
    IF BufferCmdTara = 1 AND WORD_TO_INT(PesoSiloBruto) < 50 THEN
        TaraSiloLocal_dg := 0;
    END_IF;
    BufferCmdTara := 0;
END_IF;
CmdTara := BufferCmdTara;

// 6. TEMPORIZADORES DE ESTADO
TMR_TimeoutCarga(IN := (eEstado = CARREGANDO), PT := T#15S);
TMR_Limpeza(IN := (eEstado = LIMPANDO), PT := TempoLimpeza);
TMR_Estabilizacao(IN := (eEstado = ESTABILIZANDO), PT := TempoEstabilizacao);
TMR_EsvaziarSilo(IN := (eEstado = PURGA_ESVAZIANDO_SILO), PT := TempoMaxEsvaziarSilo);
TMR_SiloVazio(IN := (eEstado = PURGA_ESVAZIANDO_SILO) AND (PesoSilo <= 15), PT := T#2500MS);
TMR_Purga(IN := (eEstado = PURGA_EXECUTANDO), PT := TempoPurga);

// 7. MÁQUINA DE ESTADOS
CASE eEstado OF
    
    PARADO:
        EstadoTexto := 'PARADO';
        CmdValvula := 0;
        AlvoTracaoEsteira := 0;
        AlvoGiroTT := 0;
        
        IF VS_IniciarDosagemA THEN
            IF PresencaCaixaA AND SiloComMaterial THEN
                CaixaAlvo := 1;
                eEstado := BUSCANDO_POSICAO;
            END_IF;
            VS_IniciarDosagemA := FALSE;
            
        ELSIF VS_IniciarDosagemB THEN
            IF PresencaCaixaB AND SiloComMaterial THEN
                CaixaAlvo := 2;
                eEstado := BUSCANDO_POSICAO;
            END_IF;
            VS_IniciarDosagemB := FALSE;

        ELSIF VS_EsvaziarSilo THEN
            IF PresencaCaixaA THEN
                FlagLimpezaCompleta := TRUE;
                eEstado := PURGA_POSICIONANDO;
            END_IF;
            VS_EsvaziarSilo := FALSE;
            
        ELSIF VS_Purga THEN
            IF PresencaCaixaA THEN
                FlagLimpezaCompleta := FALSE;
                eEstado := PURGA_POSICIONANDO;
            END_IF;
            VS_Purga := FALSE;
        END_IF;

    BUSCANDO_POSICAO:
        EstadoTexto := 'BUSCANDO POSICAO';
        IF CaixaAlvo = 1 THEN
            IF FC_CaixaA THEN
                AlvoGiroTT := 0;
                IF WORD_TO_INT(PesoCaixaA) < TaraCacambaA_dg THEN
                    TaraCacambaA_dg := WORD_TO_INT(PesoCaixaA);
                END_IF;
                PesoLiquidoCxA_dg   := WORD_TO_INT(PesoCaixaA) - TaraCacambaA_dg;
                IF PesoLiquidoCxA_dg < 0 THEN PesoLiquidoCxA_dg := 0; END_IF;

                PesoSiloInicial_dg  := PesoSilo;
                PesoDestinoIni_dg   := PesoLiquidoCxA_dg;
                TimeoutTransporte   := FALSE;
                
                AlvoDescargaSilo_dg := SetpointPeso_dg - EstoqueEsteira_dg;
                IF AlvoDescargaSilo_dg < 0 THEN AlvoDescargaSilo_dg := 0; END_IF;
                
                PontoDeCorte_dg := AlvoDescargaSilo_dg - PreAct_dg;
                IF PontoDeCorte_dg < 10 THEN PontoDeCorte_dg := 10; END_IF;
                
                IF AlvoDescargaSilo_dg <= 15 THEN
                    eEstado := LIMPANDO;
                ELSE
                    eEstado := CARREGANDO;
                END_IF;
            ELSE
                CmdGiroTT_Dir := 1;
                AlvoGiroTT := PWM_Giro_CaixaA;
            END_IF;
            
        ELSIF CaixaAlvo = 2 THEN
            IF FC_CaixaB THEN
                AlvoGiroTT := 0;
                IF WORD_TO_INT(PesoCaixaB) < TaraCacambaB_dg THEN
                    TaraCacambaB_dg := WORD_TO_INT(PesoCaixaB);
                END_IF;
                PesoLiquidoCxB_dg   := WORD_TO_INT(PesoCaixaB) - TaraCacambaB_dg;
                IF PesoLiquidoCxB_dg < 0 THEN PesoLiquidoCxB_dg := 0; END_IF;

                PesoSiloInicial_dg  := PesoSilo;
                PesoDestinoIni_dg   := PesoLiquidoCxB_dg;
                TimeoutTransporte   := FALSE;
                
                AlvoDescargaSilo_dg := SetpointPeso_dg - EstoqueEsteira_dg;
                IF AlvoDescargaSilo_dg < 0 THEN AlvoDescargaSilo_dg := 0; END_IF;
                
                PontoDeCorte_dg := AlvoDescargaSilo_dg - PreAct_dg;
                IF PontoDeCorte_dg < 10 THEN PontoDeCorte_dg := 10; END_IF;
                
                IF AlvoDescargaSilo_dg <= 15 THEN
                    eEstado := LIMPANDO;
                ELSE
                    eEstado := CARREGANDO;
                END_IF;
            ELSE
                CmdGiroTT_Dir := 0;
                AlvoGiroTT := PWM_Giro_CaixaB;
            END_IF;
        END_IF;

    CARREGANDO:
        AlvoTracaoEsteira := PWM_Cinta_Carga;
        
        PesoDosado_dg := PesoSiloInicial_dg - PesoSilo;
        IF PesoDosado_dg < 0 THEN PesoDosado_dg := 0; END_IF;
        
        IF CaixaAlvo = 1 THEN
            PesoRecebido_dg := PesoLiquidoCxA_dg - PesoDestinoIni_dg;
        ELSE
            PesoRecebido_dg := PesoLiquidoCxB_dg - PesoDestinoIni_dg;
        END_IF;
        IF PesoRecebido_dg < 0 THEN PesoRecebido_dg := 0; END_IF;
        
        IF (PesoDosado_dg >= PontoDeCorte_dg) OR 
           (PesoRecebido_dg >= (SetpointPeso_dg - OffsetQuedaEsteira_dg)) OR 
           TMR_TimeoutCarga.Q THEN
            CmdValvula := 0;
            eEstado := LIMPANDO;
            
        ELSIF PesoDosado_dg >= DoseAberturaMax_dg THEN
            EstadoTexto := 'CARREGANDO (VAZAO FINA)';
            CmdValvula  := AberturaValvulaFina;
            
        ELSE
            EstadoTexto := 'CARREGANDO (VAZAO MAX)';
            CmdValvula  := AberturaValvulaMax;
        END_IF;

    LIMPANDO:
        EstadoTexto := 'AJUSTE FINO / ESTEIRA';
        CmdValvula := 0;
        
        IF CaixaAlvo = 1 THEN
            PesoRecebido_dg := PesoLiquidoCxA_dg - PesoDestinoIni_dg;
        ELSE
            PesoRecebido_dg := PesoLiquidoCxB_dg - PesoDestinoIni_dg;
        END_IF;
        IF PesoRecebido_dg < 0 THEN PesoRecebido_dg := 0; END_IF;
        
        IF PesoRecebido_dg >= (SetpointPeso_dg - OffsetQuedaEsteira_dg) THEN
            AlvoTracaoEsteira := 0;
            CmdTracaoEsteira  := 0;
            TimeoutTransporte := FALSE;
            eEstado := ESTABILIZANDO;
            
        ELSIF TMR_Limpeza.Q THEN
            AlvoTracaoEsteira := 0;
            CmdTracaoEsteira  := 0;
            TimeoutTransporte := TRUE;
            eEstado := ESTABILIZANDO;
            
        ELSIF PesoRecebido_dg >= (SetpointPeso_dg - MargemDesacelera_dg) THEN
            AlvoTracaoEsteira := PWM_Cinta_Limpeza;
            
        ELSE
            AlvoTracaoEsteira := PWM_Cinta_Carga;
        END_IF;

    ESTABILIZANDO:
        EstadoTexto := 'ESTABILIZANDO PESO';
        CmdValvula := 0;
        AlvoTracaoEsteira := 0;
        CmdTracaoEsteira  := 0;
        
        IF TMR_Estabilizacao.Q THEN
            IF CaixaAlvo = 1 THEN
                PesoRecebido_dg := PesoLiquidoCxA_dg - PesoDestinoIni_dg;
            ELSE
                PesoRecebido_dg := PesoLiquidoCxB_dg - PesoDestinoIni_dg;
            END_IF;
            IF PesoRecebido_dg < 0 THEN PesoRecebido_dg := 0; END_IF;
            
            PesoDosado_dg := PesoSiloInicial_dg - PesoSilo;
            IF PesoDosado_dg < 0 THEN PesoDosado_dg := 0; END_IF;
            
            ErroDosagem_dg := PesoRecebido_dg - SetpointPeso_dg;
            ErroDosagem_g  := INT_TO_REAL(ErroDosagem_dg) / 10.0;
            
            IF TimeoutTransporte THEN
                EstoqueEsteira_dg := 0;
            ELSE
                EstoqueEsteira_dg := EstoqueEsteira_dg + PesoDosado_dg - PesoRecebido_dg;
                IF EstoqueEsteira_dg < 0 THEN
                    EstoqueEsteira_dg := 0;
                ELSIF EstoqueEsteira_dg > 250 THEN
                    EstoqueEsteira_dg := 250;
                END_IF;
            END_IF;
            
            IF PesoDosado_dg > PontoDeCorte_dg THEN
                VooRealSilo_dg := PesoDosado_dg - PontoDeCorte_dg;
                PreAct_dg := (PreAct_dg + VooRealSilo_dg) / 2;
                IF PreAct_dg < 10 THEN PreAct_dg := 10; END_IF;
                IF PreAct_dg > 100 THEN PreAct_dg := 100; END_IF;
            END_IF;
            
            eEstado := REVERTENDO;
        END_IF;

    REVERTENDO:
        EstadoTexto := 'CONCLUIDO';
        AlvoTracaoEsteira := 0; 
        AlvoGiroTT := 0;
        eEstado := PARADO;

    PURGA_POSICIONANDO:
        EstadoTexto := 'PURGA: POSICIONANDO';
        CmdValvula := 0;
        IF NOT PresencaCaixaA THEN
            AlvoGiroTT := 0;
            FlagLimpezaCompleta := FALSE;
            eEstado := PARADO;
        ELSIF FC_CaixaA THEN
            AlvoGiroTT := 0;
            IF FlagLimpezaCompleta THEN
                eEstado := PURGA_ESVAZIANDO_SILO;
            ELSE
                eEstado := PURGA_EXECUTANDO;
            END_IF;
        ELSE
            CmdGiroTT_Dir := 1;
            AlvoGiroTT := PWM_Giro_CaixaA;
        END_IF;

    PURGA_ESVAZIANDO_SILO:
        EstadoTexto := 'LIMPEZA: ESVAZIANDO SILO';
        CmdValvula := 100;
        AlvoGiroTT := 0;
        AlvoTracaoEsteira := PWM_Cinta_Carga;
        
        IF NOT PresencaCaixaA THEN
            CmdValvula := 0;
            AlvoTracaoEsteira := 0;
            FlagLimpezaCompleta := FALSE;
            eEstado := PARADO;
        ELSIF TMR_SiloVazio.Q OR TMR_EsvaziarSilo.Q THEN
            CmdValvula := 0;
            FlagLimpezaCompleta := FALSE;
            eEstado := PURGA_EXECUTANDO;
        END_IF;

    PURGA_EXECUTANDO:
        EstadoTexto := 'PURGA: LIMPANDO ESTEIRA';
        CmdValvula := 0;
        AlvoGiroTT := 0;
        AlvoTracaoEsteira := PWM_Cinta_Carga;
        
        IF VS_EsvaziarSilo THEN
            VS_EsvaziarSilo := FALSE;
            eEstado := PURGA_ESVAZIANDO_SILO;
        ELSIF NOT PresencaCaixaA OR TMR_Purga.Q THEN
            AlvoTracaoEsteira := 0;
            EstoqueEsteira_dg := 0;
            FlagLimpezaCompleta := FALSE;
            eEstado := PARADO;
        END_IF;

END_CASE;

// 8. RAMPAS PWM
IF AlvoTracaoEsteira = 0 THEN
    CmdTracaoEsteira := 0; 
ELSIF CmdTracaoEsteira < AlvoTracaoEsteira THEN
    CmdTracaoEsteira := CmdTracaoEsteira + PassoRampaPWM;
    IF CmdTracaoEsteira > AlvoTracaoEsteira THEN CmdTracaoEsteira := AlvoTracaoEsteira; END_IF;
ELSIF CmdTracaoEsteira > AlvoTracaoEsteira THEN
    CmdTracaoEsteira := AlvoTracaoEsteira;
END_IF;

IF AlvoGiroTT = 0 THEN
    CmdGiroTT_PWM := 0; 
ELSIF CmdGiroTT_PWM < AlvoGiroTT THEN
    CmdGiroTT_PWM := CmdGiroTT_PWM + PassoRampaPWM;
    IF CmdGiroTT_PWM > AlvoGiroTT THEN CmdGiroTT_PWM := AlvoGiroTT; END_IF;
ELSIF CmdGiroTT_PWM > AlvoGiroTT THEN
    CmdGiroTT_PWM := AlvoGiroTT;
END_IF;

// 9. SIMULAÇÃO POSIÇÃO ESTEIRA
IF FC_CaixaA THEN
    PosicaoEsteiraSimulada := 0.0;
ELSIF FC_CaixaB THEN
    PosicaoEsteiraSimulada := 100.0;
ELSIF CmdGiroTT_PWM > 0 THEN
    IF CmdGiroTT_Dir = 1 THEN
        PosicaoEsteiraSimulada := PosicaoEsteiraSimulada - 0.5;
        IF PosicaoEsteiraSimulada < 0.0 THEN PosicaoEsteiraSimulada := 0.0; END_IF;
    ELSE
        PosicaoEsteiraSimulada := PosicaoEsteiraSimulada + 0.5;
        IF PosicaoEsteiraSimulada > 100.0 THEN PosicaoEsteiraSimulada := 100.0; END_IF;
    END_IF;
END_IF;

// 10. ESCRITA MODBUS
Escrita_RTU[0] := CmdValvula;       Escrita_TCP[0] := CmdValvula;
Escrita_RTU[1] := CmdTracaoEsteira; Escrita_TCP[1] := CmdTracaoEsteira;
Escrita_RTU[2] := CmdGiroTT_PWM;    Escrita_TCP[2] := CmdGiroTT_PWM;
Escrita_RTU[3] := CmdGiroTT_Dir;    Escrita_TCP[3] := CmdGiroTT_Dir;
Escrita_RTU[4] := CmdTara;          Escrita_TCP[4] := CmdTara;
Escrita_RTU[5] := WdCodesys;        Escrita_TCP[5] := WdCodesys;
Escrita_RTU[6] := CmdModoRede;      Escrita_TCP[6] := CmdModoRede;

// 11. TRACE
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
