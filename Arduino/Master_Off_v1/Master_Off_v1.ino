#include <EEPROM.h>
#include <IRremote.h>
#include "dgio.h"
#define ARRAY_LENGTH( arr ) sizeof( arr ) / sizeof( arr[0] )

//Para Comandos via Serial
String input;
#define MAX_LENGTH 64


int Valor_Sensor_LDR = 0;//Variavel Auxiliar para leitura da porta analogica
int Valor_Sensor_Luz = 0;//Variavel Auxiliar para leitura da porta analogica
int Valor_Sensor_Som = 0;//Variavel Auxiliar para leitura da porta analogica

#define delay1 (150)
#define delay2 (1000)

typedef enum io_type {
    IO_TYPE_DIGITAL = 0,
    IO_TYPE_ANALOG
} io_type_t;

#define RECEPTOR_IR        (2) //D2 - Receptor Infravermelho
#define EMISSOR_IR         (3) //D3 - Emissor Infravermelho 
#define LED_TEMPO          (4) //D4 - Led Tempo
#define LED_EQUIPAMENTO    (6) //D5 - Led Equipamento
#define LED_SENSIBILIDADE (11) //D6 - Led Sensibilidade
#define LED_NV1            (7) //D7 - Led Nivel 1
#define LED_NV2            (8) //D8 - Led Nivel 2
#define LED_NV3            (9) //D9 - Led Nivel 3
#define LED_NV4           (10) //D11 - Led Nivel 4
#define LED_NV5           (12) //D12 - Led Nivel 5
#define Buzzer             (5) //D10 - Buzzer
#define SENSOR_SOM        (13) //D13 - Sensor de Som: saida digital

#define Sensor_LDR     (A0) //A0 - LDR
#define Pot_Luz        (A1) //A1 - Ajuste de Luz
#define Pot_Som        (A2) //A2 - Ajuste de Som
#define Botao_Cadastro (A4) //A3 - Botão Cadastro // CH1
#define Botao_Mais     (A5) //A4 - Botão +
#define Botao_Menos    (A7) //A5 - Botão -
#define Botao_Seta     (A7) //A7 tambem // (A6) //A6 - Botão ->
#define Botao_OK       (A3) //A7 - Botão OK

#define EVT_APERT_MIN_CONT  (250)
#define EVT_SEGUR_MIN_CONT (1000)

typedef enum evento_botao {
    EVENTO_NO_PRESS = 0,
    EVENTO_BOTAO_APERT,
    EVENTO_BOTAO_SEGUR,
    EVENTO_BOTAO__NUM
} evento_botao_t;

typedef enum botao {
    BOTAO_CADASTRAR = 0,
    BOTAO_MAIS,
    BOTAO_MENOS,
    BOTAO_SETA,
    BOTAO_OK,
    BOTAO__NUM
} botao_t;

#define BOTAO_CONFIG_PORT_IDX 0
#define BOTAO_CONFIG_MIN_IDX 1
#define BOTAO_CONFIG_MAX_IDX 2
#define BOTAO_CONFIG_BTN_IDX 3

static uint16_t botoes_configs[][4] = {
    {Botao_Cadastro, 0,  500, BOTAO_CADASTRAR}, //BOTAO_CADASTRAR
    {Botao_Mais,     0,  500, BOTAO_MAIS}, //BOTAO_MAIS
    {Botao_Menos,    0,  500, BOTAO_MENOS}, //BOTAO_MENOS
    {Botao_Seta,   500, 1023, BOTAO_SETA}, //BOTAO_SETA
    {Botao_OK,       0,  500, BOTAO_OK}  //BOTAO_OK
};

#define SYS_IO_TYPE_IDX   0
#define SYS_IO_NUM_IDX    1
#define SYS_IO_DIR_IDX    2
#define SYS_IO_TEST_T_IDX 3

static uint16_t sys_ios[][4] = {
    {IO_TYPE_DIGITAL, RECEPTOR_IR,       INPUT,  delay1}, 
    {IO_TYPE_DIGITAL, SENSOR_SOM,        INPUT,  delay1}, 
//    {IO_TYPE_DIGITAL, EMISSOR_IR,        OUTPUT, delay1*9}, 
    {IO_TYPE_DIGITAL, LED_TEMPO,         OUTPUT, delay1}, 
    {IO_TYPE_DIGITAL, LED_EQUIPAMENTO,   OUTPUT, delay1}, 
    {IO_TYPE_DIGITAL, LED_SENSIBILIDADE, OUTPUT, delay1}, 
    {IO_TYPE_DIGITAL, LED_NV1,           OUTPUT, delay1}, 
    {IO_TYPE_DIGITAL, LED_NV2,           OUTPUT, delay1}, 
    {IO_TYPE_DIGITAL, LED_NV3,           OUTPUT, delay1}, 
    {IO_TYPE_DIGITAL, LED_NV4,           OUTPUT, delay1}, 
    {IO_TYPE_DIGITAL, LED_NV5,           OUTPUT, delay1}, 
    {IO_TYPE_DIGITAL, Buzzer,            OUTPUT, delay2}
};

static dgio_t dg_leds[] = {
    {LED_TEMPO,         1},
    {LED_EQUIPAMENTO,   0},
    {LED_SENSIBILIDADE, 0},
    {LED_NV1,           0},
    {LED_NV2,           0},
    {LED_NV3,           0},
    {LED_NV4,           0},
    {LED_NV5,           0},
};

typedef enum sys_state {
    SYS_STATE_NORMAL = 0,
    SYS_STATE_CAPTURING,
    SYS_STATE_DESLIGANDO,
    SYS_STATE__NUM
} sys_state_t;

/* Functions Declarations */
/*    Used input Related */
evento_botao_t teste_botoes(botao_t *botao);
/*    EEPROM Related */
void zero_eeprom();
int16_t write_ir_signal(decode_results *ir_signal, uint8_t slot_idx);
int16_t read_ir_signal(decode_results *ir_signal, uint8_t slot_idx);
/*    IR Related */
void dump(decode_results *results);
/*    System State */
void change_state(sys_state_t *new_state);
static inline uint8_t ir_prot_get_idx(decode_type_t proto);
/* Private variables */
/*    Used input Related */
static botao_t curbtn;
static const char *botoes_labels[] = {
    "Cadastro",
    "Mais",
    "Menos",
    "Seta",
    "OK"
};
static evento_botao_t curbtn_evt;
static const char *eventos_labels[] = {
    "nao apertado",
    "pressionado",
    "segurado"
};
/*    EEPROM Related */
#define EEP_LENGTH (512)
#define EEP_SLOT_LENGTH (32)
#define EEP_SLOT_NUM (5)
#define EEP_SLOT_0_ADD (16)
/*    IR Related */
static IRrecv irrecv(RECEPTOR_IR);
static decode_results results;
static IRsend irsend;
#define ir_prot_enum_offset (1)
static const char *ir_protocols_str[] = {
    "UNKNOWN",
    "UNUSED",
    "RC5",
    "RC6",
    "NEC",
    "SONY",
    "PANASONIC",
    "JVC",
    "SAMSUNG",
    "WHYNTER",
    "AIWA_RC_T501",
    "LG",
    "SANYO",
    "MITSUBISHI",
    "DISH",
    "SHARP",
    "DENON",
    "PRONTO",
    "LEGO_PF",
};
#define IR_PROT_IMPL_STS_DEC_IDX (0)
#define IR_PROT_IMPL_STS_ENC_IDX (1)
static uint8_t ir_prot_impl_status[][2] = {
    {0, 0},//UNKNOWN
    {0, 0},//UNUSED
    {DECODE_RC5, SEND_RC5},
    {DECODE_RC6, SEND_RC6},
    {DECODE_NEC, SEND_NEC},
    {DECODE_SONY, SEND_SONY},
    {DECODE_PANASONIC, SEND_PANASONIC},
    {DECODE_JVC, SEND_JVC},
    {DECODE_SAMSUNG, SEND_SAMSUNG},
    {DECODE_WHYNTER, SEND_WHYNTER},
    {DECODE_AIWA_RC_T501, SEND_AIWA_RC_T501},
    {DECODE_LG, SEND_LG},
    {DECODE_SANYO, SEND_SANYO},
    {DECODE_MITSUBISHI, SEND_MITSUBISHI},
    {DECODE_DISH, SEND_DISH},
    {DECODE_SHARP, SEND_SHARP},
    {DECODE_DENON, SEND_DENON},
    {DECODE_PRONTO, SEND_PRONTO},
    {DECODE_LEGO_PF, SEND_LEGO_PF}
};
/*    Time */
// Set TIMER0_INIT_VAL to the correct value for our interrupt interval
//TIMER0_INIT_VAL = 64911;   // preload timer 65536-16MHz/256/100Hz
//TIMER0_INIT_VAL = 64286;   // preload timer 65536-16MHz/256/50Hz
//TIMER0_INIT_VAL = 34286;   // preload timer 65536-16MHz/256/2Hz
#define TIMER0_INIT_VAL (34286)
#define CAPTURE_TOUT (5)
static unsigned long tim1_ovcont = 0;
static uint16_t last_nobody_timestamp;
/*    System State */
static sys_state_t curr_state = SYS_STATE_NORMAL; 
static const char *sys_state_labels[SYS_STATE__NUM] = {
    "normal",
    "capturando",
    "desligando"
};
static unsigned int start_time;
//Ajuste forever alone
#define TURNOFF_TOUT (30) //30 ciclos de timer utilizado(timer 1 com 2hz).aproximadamente 15s

#define FLAG_PRESENCA_LUZ (0x1)
#define FLAG_PRESENCA_SOM (0x2)
static uint8_t flags_presenca;

/* Function implementations */

void dump_eeprom(){
    size_t i = 0;
    while (i < EEP_LENGTH)
    {
        Serial.print(EEPROM.read(i), HEX);
        Serial.print(", ");
        i++;
        if(i % 16 == 0){
            Serial.println("");
        }
    }
}

void zero_eeprom(){
    size_t i = 0;
    while (i < EEP_LENGTH)
    {
        EEPROM.write(i, 0);
        i++;
    }
}

static inline uint8_t ir_prot_get_idx(decode_type_t proto){
    return proto+ir_prot_enum_offset;
}

int16_t write_ir_signal(decode_results *ir_signal, uint8_t slot_idx){
    size_t current_addr = EEP_SLOT_0_ADD;
    int16_t written = 0;
    if(slot_idx >= EEP_SLOT_NUM){
        return -1;
    }
    current_addr += (slot_idx * EEP_SLOT_LENGTH);
    if((current_addr + EEP_SLOT_LENGTH) >= EEP_LENGTH){
        return -1;
    }
    EEPROM.write( current_addr, (uint8_t) ir_signal->decode_type );
    current_addr++; written++;

    EEPROM.write( current_addr,  ir_signal->bits & 0xff );
    current_addr++; written++;

    EEPROM.write( current_addr, (ir_signal->value >> 24) & 0xff );
    current_addr++; written++;
    EEPROM.write( current_addr, (ir_signal->value >> 16) & 0xff );
    current_addr++; written++;
    EEPROM.write( current_addr, (ir_signal->value >>  8) & 0xff );
    current_addr++; written++;
    EEPROM.write( current_addr,  ir_signal->value        & 0xff );
    current_addr++; written++;
    return (int16_t) written;
}

int16_t read_ir_signal(decode_results *ir_signal, uint8_t slot_idx){
    size_t current_addr = EEP_SLOT_0_ADD;
    int16_t readed = 0;
    uint8_t eep_data;
    uint8_t offset;
    size_t i;
    if(slot_idx >= EEP_SLOT_NUM){
        return -1;
    }
    current_addr += (slot_idx * EEP_SLOT_LENGTH);
    if((current_addr + EEP_SLOT_LENGTH) >= EEP_LENGTH){
        return -1;
    }

    memset(ir_signal, 0, sizeof(decode_results));//FIXME ver depois se e realmente necessario
    ir_signal->decode_type = EEPROM.read( current_addr );
    current_addr++; readed++;

    ir_signal->bits = EEPROM.read( current_addr );
    current_addr++; readed++;

    ir_signal->value = 0;
    Serial.print("Initual Value: ");    
    Serial.println(ir_signal->value, HEX);    
    Serial.println("##### READING EEPROM #####");
    for(i = 0; i < 4; i++){
        offset = 24 - (i*8);
        eep_data = EEPROM.read( current_addr );
        ir_signal->value |= ((uint32_t) eep_data) << offset;
        Serial.print("ADDR: ");    
        Serial.print(current_addr, HEX);    
        Serial.print("; Value: ");    
        Serial.print(eep_data, HEX);    
        Serial.print("; Offset: ");    
        Serial.print(offset, DEC);    
        Serial.print("; Total: ");    
        Serial.println(ir_signal->value, HEX);    
        current_addr++; readed++;
    }

    return (int16_t) readed;
}

void change_state(sys_state_t new_state){
    Serial.print("Changing state from ");
    Serial.print(sys_state_labels[curr_state]);
    Serial.print(" to ");
    Serial.println(sys_state_labels[new_state]);
    curr_state = new_state;
}

static int int_ret;
static size_t ctn;
void setup() {
    /* TIMER 1 Setup { */
    noInterrupts();           // disable all interrupts
    TCCR1A = 0;
    TCCR1B = 0;

    TCNT1 = TIMER0_INIT_VAL;   // preload timer
    TCCR1B |= (1 << CS12);    // 256 prescaler 
    TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt
    interrupts();             // enable all interrupts
    /* } TIMER 1 Setup */

    inicializar_pinos();//Configura os pinos como entrada e saída
    inicializar_serial();//Inicializa Serial e exibe informações da placa
    curbtn_evt = teste_botoes(&curbtn);//Função que testa os botões e leds da placa.
    if (curbtn_evt != EVENTO_NO_PRESS && curbtn == BOTAO_CADASTRAR){
        Serial.println("zerando EEPROM");
        zero_eeprom();
        Serial.println("zerada");
    }
    irrecv.enableIRIn(); // Start the receiver
    // teste_saidas();//Função que testa dos Leds e Buzzer
    dgio_set_timer(&tim1_ovcont);
    last_nobody_timestamp = tim1_ovcont;
    dump_eeprom();
    for(ctn = 0; ctn < EEP_SLOT_NUM; ctn++){
        int_ret = read_ir_signal(&results, ctn);
        if(int_ret < 0){
            Serial.println("Erro ao LER na EEPROM");
        }
        if(results.decode_type == UNKNOWN || results.decode_type == UNUSED ){
            dgio_write(&dg_leds[ctn+3], 0);//FIXME +3
        } else {
            dgio_write(&dg_leds[ctn+3], 1);//FIXME +3
        }
    }
}

ISR(TIMER1_OVF_vect)        // interrupt service routine 
{
  TCNT1 = TIMER0_INIT_VAL;   // preload timer
  tim1_ovcont++;
}

void enviaIR(decode_results *ir_signal){
    Serial.print("##### Sending #####");
    if (!ir_prot_impl_status[ir_signal->decode_type][IR_PROT_IMPL_STS_ENC_IDX]) {
        Serial.print("Error: Protocol ");
        Serial.print(ir_protocols_str[ir_prot_get_idx(ir_signal->decode_type)]);
        Serial.println("encode is not implemented.");
        return;
    }
    if (ir_signal->decode_type == RC5) {
        irsend.sendRC5(ir_signal->value, ir_signal->bits) ;
    } else if (ir_signal->decode_type == RC6) {
        irsend.sendRC6(ir_signal->value, ir_signal->bits) ;
    } else if (ir_signal->decode_type == NEC) {
        irsend.sendNEC(ir_signal->value, ir_signal->bits) ;
    } else if (ir_signal->decode_type == SONY) {
        irsend.sendSony(ir_signal->value, ir_signal->bits) ;
    } else if (ir_signal->decode_type == PANASONIC) {
        Serial.print("Sending PANASONIC - Address: ");
        Serial.print(ir_signal->address, HEX);
        Serial.print(" Value: ");
        irsend.sendPanasonic(ir_signal->address,  ir_signal->value) ;
    } else if (ir_signal->decode_type == JVC) {
        irsend.sendJVC(ir_signal->value, ir_signal->bits,  false) ;
    } else if (ir_signal->decode_type == SAMSUNG) {
        irsend.sendSAMSUNG(ir_signal->value, ir_signal->bits) ;
    } else if (ir_signal->decode_type == WHYNTER) {
        irsend.sendWhynter(ir_signal->value, ir_signal->bits) ;
    // } else if (ir_signal->decode_type == AIWA_RC_T501) {
    //     irsend.sendAiwaRCT501(int code) ;
    // }
    } else if (ir_signal->decode_type == LG) {
        irsend.sendLG(ir_signal->value, ir_signal->bits) ;
    // }
    //     irsend.sendSanyo( ) ; // NOT WRITTEN
    // }
    //     irsend.sendMitsubishi( ) ; // NOT WRITTEN
    // }
    } else if (ir_signal->decode_type == DISH) {
        irsend.sendDISH(ir_signal->value, ir_signal->bits) ;
    } else if (ir_signal->decode_type == SHARP) {
        Serial.print("Sending PANASONIC - Address: ");
        Serial.print(ir_signal->address, HEX);
        Serial.print(" Value: ");
        irsend.sendPanasonic(ir_signal->address,  ir_signal->value) ;
    // }
    } else if (ir_signal->decode_type == DENON) {
         irsend.sendDenon(ir_signal->value, ir_signal->bits) ;
    // }
    //     irsend.sendPronto(char* code,  bool repeat,  bool fallback) ;
    // }
    //     irsend.sendLegoPowerFunctions(uint16_t data, bool repeat = true) ;
    } else {
        Serial.print("Error!!!");
    }
    return;
}

static size_t cont;
void loop() {
    /* state independent code { */
    curbtn_evt = teste_botoes(&curbtn);//Função que testa os botões e leds da placa.
    /* rotina de deteccao de presenca de som e luz {
    if(analogRead(Sensor_LDR) > analogRead(Pot_Luz)){//Compara o valor lido pelo LDR com o do potenciometro e atua ou nao a rotina de desligamento.
        //teste_entradas();//exibe numeros das portas analogicas
        flags_presenca |= FLAG_PRESENCA_LUZ;
    } else {
      flags_presenca &= ~FLAG_PRESENCA_LUZ;
    }
    if(digitalRead(SENSOR_SOM) && analogRead(Pot_Som) > 511){//Considera e desliga
        flags_presenca |= FLAG_PRESENCA_SOM;
    } else {
        flags_presenca &= ~FLAG_PRESENCA_SOM; //MENOR Q 511 - Ignora o SOM
    }
    
    if((flags_presenca & FLAG_PRESENCA_LUZ) || 
       (flags_presenca & FLAG_PRESENCA_SOM)
    ){
        Serial.print("Presenca detectada: LUZ => ");
        Serial.print((flags_presenca & FLAG_PRESENCA_LUZ) ? "V" : "F");
        Serial.print("; SOM => ");
        Serial.print((flags_presenca & FLAG_PRESENCA_SOM) ? "V" : "F");
        teste_entradas();//exibe numeros das portas analogicas
        last_nobody_timestamp = tim1_ovcont;
    } else {
        Serial.print("Clocks para desligar: ");
        Serial.println(TURNOFF_TOUT - (tim1_ovcont - last_nobody_timestamp));
        //beep();
    }
    } *///////////////
    
    dgio_update_arr(dg_leds, ARRAY_LENGTH(dg_leds));
    /* } */
    if(curr_state == SYS_STATE_NORMAL){
        if (curbtn_evt != EVENTO_NO_PRESS){
            if(curbtn >= 0 &&  curbtn < EEP_SLOT_NUM){
                if (curbtn_evt == EVENTO_BOTAO_SEGUR){
                    change_state(SYS_STATE_CAPTURING);
                } else if (curbtn_evt == EVENTO_BOTAO_APERT){
                    Serial.println("#### Enviando ####");
                    int_ret = read_ir_signal(&results, curbtn);
                    if(int_ret < 0){
                        Serial.println("Erro ao LER na EEPROM");
                    } else {
                        dump(&results);
                        enviaIR(&results);
                        beep();
                    }
                } else {
                    Serial.println("Error");
                }
            } else {
                Serial.println("Error");
            }
        } else {// Quando esta no estado Normal e ninguem apertou nenhum botao
            if((tim1_ovcont - last_nobody_timestamp) > TURNOFF_TOUT){//Se o tempo que nao tem ninguem e maior que o timeout de desligar
                // change_state(SYS_STATE_DESLIGANDO);//muda o estado do sistema para desligando
                last_nobody_timestamp = tim1_ovcont;
            }
        }
    } else if(curr_state == SYS_STATE_CAPTURING){
        Serial.println("#### Capturando ####");
        start_time = tim1_ovcont;
        do {
            int_ret = irrecv.decode(&results);
        } while (!int_ret && (tim1_ovcont - start_time) <= CAPTURE_TOUT );
        if((tim1_ovcont - start_time) > CAPTURE_TOUT){
            Serial.println("TIMEOUT");
            memset(&results, 0, sizeof(decode_results));
            int_ret = write_ir_signal(&results, curbtn);
            if(int_ret < 0){
                Serial.println("Erro ao ESCREVER na EEPROM");
            } else {
                dgio_write(&dg_leds[curbtn+3], 0);//FIXME +3
            }
        } else {
            dgio_write(&dg_leds[curbtn+3], 1);//FIXME +3
            Serial.println(results.value, HEX);
            dump(&results);
            int_ret = write_ir_signal(&results, curbtn);
            if(int_ret < 0){
                Serial.println("Erro ao ESCREVER na EEPROM");
            }
            dump_eeprom();
            irrecv.resume(); // Receive the next value
        }
        change_state(SYS_STATE_NORMAL);
    } else if(curr_state == SYS_STATE_DESLIGANDO) {
        for(cont = 0; cont < EEP_SLOT_NUM; cont++){
            int_ret = read_ir_signal(&results, cont);
            if(int_ret < 0){
                Serial.println("Erro ao LER na EEPROM");
            } else {
                dump(&results);
                enviaIR(&results);
            }
            delay(1000);
        }
        change_state(SYS_STATE_NORMAL);
    } else {
        Serial.println("Error");
    }
    serial_Comandos();
}

void dump(decode_results *results) {
    // Dumps out the decode_results structure.
    // Call this after IRrecv::decode()
    int count = results->rawlen;
    Serial.print("Protocol: ");
    Serial.println(ir_protocols_str[ir_prot_get_idx(results->decode_type)]);
    if (results->decode_type == PANASONIC || results->decode_type == SHARP) {
        Serial.print("Address: ");
        Serial.println(results->address, HEX);
    }

    Serial.print("Value: ");
    Serial.println(results->value, HEX);
    Serial.print("Bits: ");
    Serial.println(results->bits, DEC);
    Serial.print("Raw (");
    Serial.print(count, DEC);
    Serial.print("): ");

    for (int i = 1; i < count; i++) {
        if (i & 1) {
            Serial.print(results->rawbuf[i]*USECPERTICK, DEC);
        }
        else {
            Serial.write('-');
            Serial.print((unsigned long) results->rawbuf[i]*USECPERTICK, DEC);
        }
        Serial.print(" ");
    }
    Serial.println();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void teste_saidas(){  
    size_t i;
    for(i=0; i < ARRAY_LENGTH(sys_ios); i++){
        if(sys_ios[i][SYS_IO_TYPE_IDX] == IO_TYPE_DIGITAL &&
           sys_ios[i][SYS_IO_DIR_IDX] == OUTPUT)
        {
            digitalWrite(sys_ios[i][SYS_IO_NUM_IDX], HIGH);
            // delay (sys_ios[i][SYS_IO_TEST_T_IDX]);
            digitalWrite(sys_ios[i][SYS_IO_NUM_IDX], LOW); 
        }
    }
}

void teste_entradas(){
    Valor_Sensor_LDR = analogRead(Sensor_LDR);
    Valor_Sensor_Luz = analogRead(Pot_Luz);
    Valor_Sensor_Som = analogRead(Pot_Som);
    Serial.print(" Sensores LDR=");
    Serial.print(Valor_Sensor_LDR);
    Serial.print(" Potenc.Luz=");
    Serial.print(Valor_Sensor_Luz);
    Serial.print(" Potenc.Som=");
    Serial.println(Valor_Sensor_Som);
}

void inicializar_serial(){
    Serial.begin(9600);//Inicializa a comunicação Serial
    input.reserve(MAX_LENGTH + 1);  // Define o tamanho máximo do buffer (+ 1 por causa do \0 no final
    Serial.println("Freedom Engenharia\n"
    "Firmware: Master Off\n"
    "Versao:   v1.\n\n"
    "Sistema Inicializado!\n"); 
    beep();
    beep();
    beep();
}

void inicializar_pinos(){
    size_t i=0;
    for(i=0; i < ARRAY_LENGTH(sys_ios); i++){
        if(sys_ios[i][SYS_IO_TYPE_IDX] == IO_TYPE_DIGITAL){
            pinMode(sys_ios[i][SYS_IO_NUM_IDX], sys_ios[i][SYS_IO_DIR_IDX]);   
        }
    }
}

void beep(){
    digitalWrite(Buzzer, HIGH);
    delay(delay1);
    digitalWrite(Buzzer, LOW);
    delay(delay1);
}

evento_botao_t teste_botoes(botao_t *botao){
    size_t i;
    uint16_t analog_read_val;
    uint16_t press_count;
    evento_botao_t evt = EVENTO_NO_PRESS; 
    for(i = 0; i < BOTAO__NUM ; i++){
        analog_read_val = analogRead(botoes_configs[i][BOTAO_CONFIG_PORT_IDX]);
        press_count = 0;
        while(botoes_configs[i][BOTAO_CONFIG_MIN_IDX] <= analog_read_val && 
           analog_read_val < botoes_configs[i][BOTAO_CONFIG_MAX_IDX])
        {
            press_count++;
            analog_read_val = analogRead(botoes_configs[i][BOTAO_CONFIG_PORT_IDX]);
            delay(1);
        }
        if(press_count == 0){
            continue;
        }
        if( 0 < press_count && press_count <= EVT_APERT_MIN_CONT ){
            continue; //aperto muito rapido...
        } else if( EVT_APERT_MIN_CONT < press_count && press_count <= EVT_SEGUR_MIN_CONT){
            evt = EVENTO_BOTAO_APERT;
            *botao = (botao_t)botoes_configs[i][BOTAO_CONFIG_BTN_IDX];
        } else if( EVT_SEGUR_MIN_CONT < press_count ){
            evt = EVENTO_BOTAO_SEGUR;
            *botao = (botao_t)botoes_configs[i][BOTAO_CONFIG_BTN_IDX];
        } else {
            Serial.print("Error at line:");
            Serial.println(__LINE__, DEC);
        }
        Serial.print("Botao ");
        Serial.print(botoes_labels[curbtn]);
        Serial.print(": ");
        Serial.print(press_count, DEC);
        Serial.print("=> ");
        Serial.println(eventos_labels[curbtn_evt]);
    }
    return evt; 
}

void serial_Comandos(){
    if (Serial.available()) {
        char c = Serial.read(); // Recebe um caracter
        if(c == '\n') {     // Se foi digitado um ENTER entao processa a String       
            Serial.println("A string digitada foi ");
            Serial.println(input);

            input.toUpperCase();    // Converte toda a String para maiusculo
            input.trim();           // Tira os espacos antes e depois
            curbtn=input.substring(1).toInt()-1;
            if(curbtn >= BOTAO__NUM){//Protecao para nao digitar numeros maior que 5
                input = "";// Limpa a String para comecar a armazenar de novo   
                return; //FIXME inform error
            }
            if(input.startsWith("W")){
                change_state(SYS_STATE_CAPTURING);
            } 
            else if (input.startsWith("E")) {
                Serial.println("#### Enviando ####");
                int_ret = read_ir_signal(&results, curbtn);
                if(int_ret < 0){
                    Serial.println("Erro ao LER na EEPROM");
                } else {
                    dump(&results);
                    enviaIR(&results);
                    beep();
                }
            } else {
                input = "";// Limpa a String para comecar a armazenar de novo   
                return; //FIXME inform error
                
            }
            input = "";// Limpa a String para comecar a armazenar de novo         
        } 
        else {    
            // Se nao veio um ENTER entao vai armazenando até o tamanho maximo que a string pode suportar        
            if(input.length() < MAX_LENGTH) {
                if((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9')){ // Aqui estamos filtrando so as letras. Poderia filtrar numeros e espaco por exemplo
                    input += c;
                }
            }
        }
    } 
}
