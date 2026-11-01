# Bancada Didática de Baixo Custo para Armazenagem, Dosagem e Empilhamento de Granulados

Repositório oficial dos códigos-fonte e arquivos de integração do Projeto Final de Curso (PFC) do curso de Engenharia de Controle e Automação da Universidade Federal de Minas Gerais (UFMG).

## 📋 Sobre o Projeto
Este projeto desenvolve e valida experimentalmente uma arquitetura de controle distribuído utilizando um **Soft-PLC** (Raspberry Pi 4 com CODESYS) e uma unidade remota de I/O inteligente (ESP32 com FreeRTOS). O objetivo principal é demonstrar o impacto físico de falhas temporais de preempção do sistema operacional (`PREEMPT_RT` vs. Linux Convencional) sobre um processo de dosagem por perda de peso (*Loss-in-Weight*).

## 📂 Estrutura do Repositório
- `esp32-firmware/`: Código em C++ desenvolvido no Arduino IDE para o ESP32, contendo o tratamento das células de carga (`HX711`), cascata de filtragem digital DSP assimétrica e pilha Modbus RTU/TCP.
- `codesys-plc/`: Algoritmo em Texto Estruturado (ST) da máquina de estados do processo de dosagem, rampa de aceleração (*soft-start*) e compensação adaptativa de material em voo (*Pre-Act*).
- `raspberry-pi-scripts/`: Serviço em Python responsável pelo monitoramento das chaves físicas do painel GPIO e acionamento do teste de estresse do sistema (`stress-ng`).

## 🛠️ Tecnologias e Hardware Utilizados
- **Controlador Mestre:** Raspberry Pi 4 Model B (Quad-Core ARM Cortex-A72)
- **Sistema Operacional / Runtime:** Raspberry Pi OS com *kernel* de tempo real (`PREEMPT_RT`) e CODESYS Control Runtime
- **Unidade Remota:** ESP32 DevKit V1 (Dual-Core Xtensa LX6) rodando FreeRTOS
- **Instrumentação:** Células de carga de 1 kg com módulos condicionadores HX711 (24-bit)
- **Atuadores:** Servomotor (comporta guilhotina), Motor CC da esteira (TT) e Micromotor de giro (N20)
- **Comunicação:** Barramento industrial RS-485 (Modbus RTU) e Rede Wi-Fi (Modbus TCP)

## 👤 Autor
* **Pedro Henrique Silva Lima**
* **Orientador:** Prof. Dr. Leandro Freitas de Abreu
* **Instituição:** Universidade Federal de Minas Gerais (UFMG) - Departamento de Engenharia Elétrica (DELT)
