/*
 * PMU.c
 *
 *  Created on: 16 de fev. de 2022
 *      Author: moreto
 */

//#include "esp_log.h"
#include <stdio.h>
#include <string.h>
#include "PMU.h"


/* Global variables: */

#ifdef ENABLE_HARMONICS
unsigned char ucData[832];	// PMU frame array considering harmonic phasors
float harmonics_R_mag[15];
float harmonics_R_phase[15];
float harmonics_S_mag[15];
float harmonics_S_phase[15];
float harmonics_T_mag[15];
float harmonics_T_phase[15];
#else
unsigned char ucData[128];
#endif

volatile float Mag_R_final,Mag_S_final,Mag_T_final,Fase_R_final,Fase_S_final,Fase_T_final;
float Freq_final;

uint16_t CRC_CCITT;

union {
	float pf;
	unsigned char byte[4];
}convert_float_to_char;

uint16_t PMU_config_frame_init(uint16_t pmuid, uint8_t config, uint32_t SOC, uint32_t FracSec){
	char CHName[16];
	unsigned long Time_base = TIME_BASE;   // 1 000 000 us
	memset(ucData, 0x00, sizeof(ucData)); // Clear array
	// 1. SYNC = Data Message Sync Byte andFrame Type
		ucData[0] = A_SYNC_AA;
		ucData[1] = config;

		// 2. FRAMESIZE = Frame size, including CHK
		#ifdef ENABLE_HARMONICS
		ucData[2] = (unsigned char)0x03;
		ucData[3] = (unsigned char)0x24; //
		#else
		ucData[2] = (unsigned char)0x00;
		ucData[3] = (unsigned char)0x72; //
		#endif

		// 3. IDCODE = ID da fonte de transmissao
		ucData[4] = (unsigned char)((pmuid & 0xFF00) >> 8);
		ucData[5] = (unsigned char)(pmuid & 0x00FF);

		// 4. SOC = Second Of Century = Estampa de tempo, segundo secular
		ucData[6] = (unsigned char)((SOC & 0xFF000000) >> 24);
		ucData[7] = (unsigned char)((SOC & 0x00FF0000) >> 16);
		ucData[8] = (unsigned char)((SOC & 0x0000FF00) >> 8);
		ucData[9] = (unsigned char)(SOC & 0x000000FF);

		// 5. FRACSEC = Fracao do segundo e qualidade do tempo
		ucData[10] = (unsigned char)((FracSec & 0xFF000000) >> 24);  //Time quality (secao 6.2.2)
		ucData[11] = (unsigned char)((FracSec & 0x00FF0000) >> 16);  //fracsec
		ucData[12] = (unsigned char)((FracSec & 0x0000FF00) >> 8);   //fracsec
		ucData[13] = (unsigned char)(FracSec & 0x000000FF);			//fracsec

		// 6. TIME_BASE = Resolucao do FRACSEC
		ucData[14] = (unsigned char)((Time_base & 0xFF000000) >> 24);
		ucData[15] = (unsigned char)((Time_base & 0x00FF0000) >> 16);
		ucData[16] = (unsigned char)((Time_base & 0x0000FF00) >> 8);
		ucData[17] = (unsigned char)(Time_base & 0x000000FF);

		// 7. NUM_PMU = numero de PMUs inclusas no frame de dados
		#ifdef ENABLE_HARMONICS
		ucData[18] = 0x00;
		ucData[19] = 0x04;
		#else
		ucData[18] = 0x00;
		ucData[19] = 0x01;
		#endif

		// 8. STN = Station Name
		ucData[20] = 'F';
		ucData[21] = 'r';
		ucData[22] = 'e';
		ucData[23] = 'e';
		ucData[24] = 'P';
		ucData[25] = 'M';
		ucData[26] = 'U';
		ucData[27] = 48 + (pmuid/100);
		ucData[28] = 48 + ((pmuid%100)/10);
		ucData[29] = 48 + ((pmuid%100)%10);
		ucData[30] = ' ';
		ucData[31] = 'R';
		ucData[32] = 'S';
		ucData[33] = 'T';
		ucData[34] = ' ';
		ucData[35] = ' ';

		// 9. IDCODE = uma PMU, nesse caso igual ao campo 3
		ucData[36] = (unsigned char)((pmuid & 0xFF00) >> 8);
		ucData[37] = (unsigned char)(pmuid & 0x00FF);

		// 10. FORMAT = Formato dos dados nos frames de dados
		ucData[38] = 0x00;
		ucData[39] = 0x0F;  // 0111b

		// 11. PHNMR = Numero de fasores
		ucData[40] = 0x00;
		ucData[41] = 0x03;  // 3 fasores

		// 12. ANNMR = Number of Analog Values
		ucData[42] = 0x00;
		ucData[43] = 0x00;  //

		// 13. DGNMR = Number of Digital Status Words
		ucData[44] = 0x00;
		ucData[45] = 0x00;

		// 14. CHNAM = Nome de fasores e canais
		memset(CHName, 0x00, 16);
		strcpy(CHName, "Phase_R");
		memcpy(ucData + 46, CHName, 16);

		memset(CHName, 0x00, 16);
		strcpy(CHName, "Phase_S");
		memcpy(ucData + 62, CHName, 16);

		memset(CHName, 0x00, 16);
		strcpy(CHName, "Phase_T");
		memcpy(ucData + 78, CHName, 16);

		uint16_t i = 94;

		// 15.  PHUNIT = fator de conversao pra canais fasoriais
		// 4 bytes pra cada fasor
		ucData[i++] = 0x00;  // 0 = Voltage, 1 = Current
		ucData[i++] = 0x00;  // ignore
		ucData[i++] = 0x00;  // ignore
		ucData[i++] = 0x00;  // ignore

		ucData[i++] = 0x00;  //
		ucData[i++] = 0x00;  //
	    ucData[i++] = 0x00;  //
		ucData[i++] = 0x00;  //

		ucData[i++] = 0x00;
		ucData[i++] = 0x00;
		ucData[i++] = 0x00;  //
		ucData[i++] = 0x00;  //

		// 18. FNOM = Frequencia nominal
		ucData[i++] = 0x00;

#if(NOMINAL_FREQ == 50)
			ucData[i++] = 0x01; // 1 = 50 Hz
#else
			ucData[i++] = 0x00; // 0 = 60 Hz
#endif

		// 19. CFGCNT = contador de alteracao da configuracao
		ucData[i++] = 0x00;
		ucData[i++] = 0x00;  //


#ifdef ENABLE_HARMONICS
		// Phase R
		unsigned char pmuid_tmp = pmuid;
		pmuid_tmp++;

		// 8. STN = Nome da estacao
		ucData[i++] = 'F';
		ucData[i++] = 'r';
		ucData[i++] = 'e';
		ucData[i++] = 'e';
		ucData[i++] = 'P';
		ucData[i++] = 'M';
		ucData[i++] = 'U';
		ucData[i++] = ' ';
		ucData[i++] = 48 + (pmuid_tmp/100);
		ucData[i++] = 48 + ((pmuid_tmp%100)/10);
		ucData[i++] = 48 + ((pmuid_tmp%100)%10);
		ucData[i++] = ' ';
		ucData[i++] = 'H';
		ucData[i++] = 'R';
		ucData[i++] = ' ';
		ucData[i++] = ' ';

		// 9. IDCODE = uma PMU, nesse caso igual ao campo 3
		ucData[i++] = (unsigned char)((pmuid_tmp & 0xFF00) >> 8);
		ucData[i++] = (unsigned char)(pmuid_tmp & 0x00FF);

		// 10. FORMAT = Formato dos dados nos frames de dados
		ucData[i++] = 0x00;
		ucData[i++] = 0x0F;  // 0111b

		// 11. PHNMR = Numero de fasores
		ucData[i++] = 0x00;
		ucData[i++] = 0x0A;  // 10 fasores

		// 12. ANNMR = Number of Analog Values
		ucData[i++] = 0x00;
		ucData[i++] = 0x00;  //

		// 13. DGNMR = Number of Digital Status Words
		ucData[i++] = 0x00;
		ucData[i++] = 0x00;

		// 14. CHNAM = Nome de fasores e canais
		uint8_t j;
		for (j = 2; j<12; j++){
			memset(CHName, 0x00, 16);
			sprintf(CHName, "%d Harmonic R", j);
			memcpy(ucData + i, CHName, 16);
			i += 16;
		}

		// 15.  PHUNIT = fator de conversao pra canais fasoriais
		// 4 bytes pra cada fasor
		for (j = 0; j<10; j++){
			ucData[i++] = 0x00;  // 0 = Voltage, 1 = Current
			ucData[i++] = 0x00;  // ignore
			ucData[i++] = 0x00;  // ignore
			ucData[i++] = 0x00;  // ignore
		}


		// 18. FNOM = Frequencia nominal
		ucData[i++] = 0x00;

#if(NOMINAL_FREQ == 50)
			ucData[i++] = 0x01; // 1 = 50 Hz
#else
			 ucData[i++] = 0x00; // 0 = 60 Hz
#endif

		// 19. CFGCNT = contador de alteracao da configuracao
		ucData[i++] = 0x00;
		ucData[i++] = 0x00;  //

		// Phase S
		pmuid_tmp++;

		// 8. STN = Nome da estacao
		ucData[i++] = 'F';
		ucData[i++] = 'r';
		ucData[i++] = 'e';
		ucData[i++] = 'e';
		ucData[i++] = 'P';
		ucData[i++] = 'M';
		ucData[i++] = 'U';
		ucData[i++] = ' ';
		ucData[i++] = 48 + (pmuid_tmp/100);
		ucData[i++] = 48 + ((pmuid_tmp%100)/10);
		ucData[i++] = 48 + ((pmuid_tmp%100)%10);
		ucData[i++] = ' ';
		ucData[i++] = 'H';
		ucData[i++] = 'S';
		ucData[i++] = ' ';
		ucData[i++] = ' ';

		// 9. IDCODE = uma PMU, nesse caso igual ao campo 3
		ucData[i++] = (unsigned char)((pmuid_tmp & 0xFF00) >> 8);
		ucData[i++] = (unsigned char)(pmuid_tmp & 0x00FF);

		// 10. FORMAT = Formato dos dados nos frames de dados
		ucData[i++] = 0x00;
		ucData[i++] = 0x0F;  // 0111b

		// 11. PHNMR = Numero de fasores
		ucData[i++] = 0x00;
		ucData[i++] = 0x0A;  // 10 fasores

		// 12. ANNMR = Number of Analog Values
		ucData[i++] = 0x00;
		ucData[i++] = 0x00;  //

		// 13. DGNMR = Number of Digital Status Words
		ucData[i++] = 0x00;
		ucData[i++] = 0x00;

		// 14. CHNAM = Nome de fasores e canais
		for (j = 2; j<12; j++){
			memset(CHName, 0x00, 16);
			sprintf(CHName, "%d Harmonic S", j);
			memcpy(ucData + i, CHName, 16);
			i += 16;
		}

		// 15.  PHUNIT = fator de conversao pra canais fasoriais
		// 4 bytes pra cada fasor
		for (j = 0; j<10; j++){
			ucData[i++] = 0x00;  // 0 = Voltage, 1 = Current
			ucData[i++] = 0x00;  // ignore
			ucData[i++] = 0x00;  // ignore
			ucData[i++] = 0x00;  // ignore
		}


		// 18. FNOM = Frequencia nominal
		ucData[i++] = 0x00;

#if(NOMINAL_FREQ == 50)
			ucData[i++] = 0x01; // 1 = 50 Hz
#else
			 ucData[i++] = 0x00; // 0 = 60 Hz
#endif

		// 19. CFGCNT = contador de alteracao da configuracao
		ucData[i++] = 0x00;
		ucData[i++] = 0x00;  //

		// Phase T
		pmuid_tmp++;

		// 8. STN = Nome da estacao
		ucData[i++] = 'F';
		ucData[i++] = 'r';
		ucData[i++] = 'e';
		ucData[i++] = 'e';
		ucData[i++] = 'P';
		ucData[i++] = 'M';
		ucData[i++] = 'U';
		ucData[i++] = ' ';
		ucData[i++] = 48 + (pmuid_tmp/100);
		ucData[i++] = 48 + ((pmuid_tmp%100)/10);
		ucData[i++] = 48 + ((pmuid_tmp%100)%10);
		ucData[i++] = ' ';
		ucData[i++] = 'H';
		ucData[i++] = 'T';
		ucData[i++] = ' ';
		ucData[i++] = ' ';

		// 9. IDCODE = uma PMU, nesse caso igual ao campo 3
		ucData[i++] = (unsigned char)((pmuid_tmp & 0xFF00) >> 8);
		ucData[i++] = (unsigned char)(pmuid_tmp & 0x00FF);

		// 10. FORMAT = Formato dos dados nos frames de dados
		ucData[i++] = 0x00;
		ucData[i++] = 0x0F;  // 0111b

		// 11. PHNMR = Numero de fasores
		ucData[i++] = 0x00;
		ucData[i++] = 0x0A;  // 10 fasores

		// 12. ANNMR = Number of Analog Values
		ucData[i++] = 0x00;
		ucData[i++] = 0x00;  //

		// 13. DGNMR = Number of Digital Status Words
		ucData[i++] = 0x00;
		ucData[i++] = 0x00;

		// 14. CHNAM = Nome de fasores e canais
		for (j = 2; j<12; j++){
			memset(CHName, 0x00, 16);
			sprintf(CHName, "%d Harmonic T", j);
			memcpy(ucData + i, CHName, 16);
			i += 16;
		}

		// 15.  PHUNIT = fator de conversao pra canais fasoriais
		// 4 bytes pra cada fasor
		for (j = 0; j<10; j++){
			ucData[i++] = 0x00;  // 0 = Voltage, 1 = Current
			ucData[i++] = 0x00;  // ignore
			ucData[i++] = 0x00;  // ignore
			ucData[i++] = 0x00;  // ignore
		}

		// 18. FNOM = Frequencia nominal
		ucData[i++] = 0x00;

#if(NOMINAL_FREQ == 50)
			ucData[i++] = 0x01; // 1 = 50 Hz
#else
			 ucData[i++] = 0x00; // 0 = 60 Hz
#endif

		// 19. CFGCNT = contador de alteracao da configuracao
		ucData[i++] = 0x00;
		ucData[i++] = 0x00;  //
#endif

		// 20. DATA_RATE = Taxa de transmissao de fasores
		ucData[i++] = ((PMU_FPS & 0xFF00) >> 8);
		ucData[i++] = (PMU_FPS & 0x00FF);

		//21. CRC-CCITT = Calcula o CRC
		CRC_CCITT = ComputeCRC(ucData, i);

		ucData[i++] = (unsigned char)((CRC_CCITT & 0xFF00) >> 8);
		ucData[i++] = (unsigned char)(CRC_CCITT & 0x00FF);

		return i;
}


uint16_t PMU_data_frame_update(uint16_t pmuid, uint8_t config, uint32_t SOC, uint32_t FracSec){
	//unsigned long Time_base = TIME_BASE;   // 1 000 000 us
	uint16_t i=0;

#ifdef ENABLE_HARMONICS
	memset(ucData, 0, 320); // Clear array
#else
	memset(ucData, 0, 50);
#endif
	memset(ucData, 0x00, sizeof(ucData)); // Clear array
	// 1. SYNC = Data Message Sync Byte andFrame Type
	ucData[i++] = A_SYNC_AA;
	ucData[i++] = config;

	// 2. FRAMESIZE = Frame size, including CHK
	#ifdef ENABLE_HARMONICS
	ucData[i++] = (unsigned char)0x01;
	ucData[i++] = (unsigned char)0x40; //
	#else
	ucData[i++] = (unsigned char)0x00;
	ucData[i++] = (unsigned char)0x73; //
	#endif

	// 3. IDCODE = ID da fonte de transmissao
	ucData[i++] = (unsigned char)((pmuid & 0xFF00) >> 8);   //4
	ucData[i++] = (unsigned char)(pmuid & 0x00FF);   //5

	// 4. SOC = Second Of Century = Estampa de tempo, segundo secular
	ucData[i++] = (unsigned char)((SOC & 0xFF000000) >> 24);   //6
	ucData[i++] = (unsigned char)((SOC & 0x00FF0000) >> 16);   //7
	ucData[i++] = (unsigned char)((SOC & 0x0000FF00) >> 8);   //8
	ucData[i++] = (unsigned char)(SOC & 0x000000FF);   //9

	// 5. FRACSEC = Fracao do segundo e qualidade do tempo
	ucData[i++] = (unsigned char)((FracSec & 0xFF000000) >> 24);  //10 - Time quality (secao 6.2.2)
	ucData[i++] = (unsigned char)((FracSec & 0x00FF0000) >> 16);  //11 - fracsec
	ucData[i++] = (unsigned char)((FracSec & 0x0000FF00) >> 8);   //12 - fracsec
	ucData[i++] = (unsigned char)(FracSec & 0x000000FF);		  //13 - fracsec

	// 6. STAT = Flags, a criterio do usuario
	ucData[i++] = 0x00;   //14
	ucData[i++] = 0x00;   //15

	convert_float_to_char.pf = Mag_R_final;
	ucData[i++] = convert_float_to_char.byte[3];	//16
	ucData[i++] = convert_float_to_char.byte[2];	//17
	ucData[i++] = convert_float_to_char.byte[1];	//18
	ucData[i++] = convert_float_to_char.byte[0];	//19

	convert_float_to_char.pf = 0.78539;// Fase_R_rad;
	ucData[i++] = convert_float_to_char.byte[3];	//20
	ucData[i++] = convert_float_to_char.byte[2];	//21
	ucData[i++] = convert_float_to_char.byte[1];	//22
	ucData[i++] = convert_float_to_char.byte[0];	//23

	convert_float_to_char.pf = Mag_S_final;
	ucData[i++] = convert_float_to_char.byte[3];	//24
	ucData[i++] = convert_float_to_char.byte[2];	//25
	ucData[i++] = convert_float_to_char.byte[1];	//26
	ucData[i++] = convert_float_to_char.byte[0];	//27

	convert_float_to_char.pf = 1.57079;//120Fase_S_rad;
	ucData[i++] = convert_float_to_char.byte[3];	//28
	ucData[i++] = convert_float_to_char.byte[2];	//29
	ucData[i++] = convert_float_to_char.byte[1];	//30
	ucData[i++] = convert_float_to_char.byte[0];	//31

	convert_float_to_char.pf = Mag_T_final;
	ucData[i++] = convert_float_to_char.byte[3];	//32
	ucData[i++] = convert_float_to_char.byte[2];	//33
	ucData[i++] = convert_float_to_char.byte[1];	//34
	ucData[i++] = convert_float_to_char.byte[0];	//35

	convert_float_to_char.pf = -1.57079;//Fase_T_rad;
	ucData[i++] = convert_float_to_char.byte[3];	//36
	ucData[i++] = convert_float_to_char.byte[2];	//37
	ucData[i++] = convert_float_to_char.byte[1];	//38
	ucData[i++] = convert_float_to_char.byte[0];	//39

	// 8. FREQ = Desvio de frequencia do nominal, em mHz
	convert_float_to_char.pf = Freq_final;
	ucData[i++] = convert_float_to_char.byte[3];	//80
	ucData[i++] = convert_float_to_char.byte[2];	//81
	ucData[i++] = convert_float_to_char.byte[1];	//82
	ucData[i++] = convert_float_to_char.byte[0];	//83

	// 9. DFREQ = ROCOF in Hz/s "times 100"
	//ROCOF = ROCOF*100;

	convert_float_to_char.pf = 0.0;//(media_rocof*100);
	ucData[i++] = convert_float_to_char.byte[3];	//84
	ucData[i++] = convert_float_to_char.byte[2];	//85
	ucData[i++] = convert_float_to_char.byte[1];	//86
	ucData[i++] = convert_float_to_char.byte[0];	//87

#ifdef ENABLE_HARMONICS
	// Phase R
	// 6. STAT = Flags, a criterio do usuario
	ucData[i++] = 0x00;   //14
	ucData[i++] = 0x00;   //15

	// 7. PHASORS
	for (int j = 0; j<10; j++){
		convert_float_to_char.pf = 0.1*j;//harmonics_R_mag[j];
		ucData[i++] = convert_float_to_char.byte[3];
		ucData[i++] = convert_float_to_char.byte[2];
		ucData[i++] = convert_float_to_char.byte[1];
		ucData[i++] = convert_float_to_char.byte[0];

		convert_float_to_char.pf = 0.0;//harmonics_R_phase[j];
		ucData[i++] = convert_float_to_char.byte[3];
		ucData[i++] = convert_float_to_char.byte[2];
		ucData[i++] = convert_float_to_char.byte[1];
		ucData[i++] = convert_float_to_char.byte[0];
	}

	// 8. FREQ = Desvio de frequencia do nominal, em mHz
	//Freq_final = (Freq_final - f0)*1000;

	convert_float_to_char.pf = 60.05;//(Freq_final);
	ucData[i++] = convert_float_to_char.byte[3];	//80
	ucData[i++] = convert_float_to_char.byte[2];	//81
	ucData[i++] = convert_float_to_char.byte[1];	//82
	ucData[i++] = convert_float_to_char.byte[0];	//83

	// 9. DFREQ = ROCOF in Hz/s "times 100"
	//ROCOF = ROCOF*100;

	convert_float_to_char.pf = 0.0;//(media_rocof*100);
	ucData[i++] = convert_float_to_char.byte[3];	//84
	ucData[i++] = convert_float_to_char.byte[2];	//85
	ucData[i++] = convert_float_to_char.byte[1];	//86
	ucData[i++] = convert_float_to_char.byte[0];	//87

	// Phase S
	// 6. STAT = Flags, a criterio do usuario
	ucData[i++] = 0x00;   //14
	ucData[i++] = 0x00;   //15

	// 7. PHASORS
	for (int j = 0; j<10; j++){
		convert_float_to_char.pf = 0.1 *j;//harmonics_S_mag[j];
		ucData[i++] = convert_float_to_char.byte[3];
		ucData[i++] = convert_float_to_char.byte[2];
		ucData[i++] = convert_float_to_char.byte[1];
		ucData[i++] = convert_float_to_char.byte[0];

		convert_float_to_char.pf = 0.0;// harmonics_S_phase[j];
		ucData[i++] = convert_float_to_char.byte[3];
		ucData[i++] = convert_float_to_char.byte[2];
		ucData[i++] = convert_float_to_char.byte[1];
		ucData[i++] = convert_float_to_char.byte[0];
	}

	// 8. FREQ = Desvio de frequencia do nominal, em mHz
	//Freq_final = (Freq_final - f0)*1000;

	convert_float_to_char.pf = 60.05;//(Freq_final);
	ucData[i++] = convert_float_to_char.byte[3];	//80
	ucData[i++] = convert_float_to_char.byte[2];	//81
	ucData[i++] = convert_float_to_char.byte[1];	//82
	ucData[i++] = convert_float_to_char.byte[0];	//83

	// 9. DFREQ = ROCOF in Hz/s "times 100"
	//ROCOF = ROCOF*100;

	convert_float_to_char.pf = 0.0;//(media_rocof*100);
	ucData[i++] = convert_float_to_char.byte[3];	//84
	ucData[i++] = convert_float_to_char.byte[2];	//85
	ucData[i++] = convert_float_to_char.byte[1];	//86
	ucData[i++] = convert_float_to_char.byte[0];	//87

	// Phase T
	// 6. STAT = Flags, a criterio do usuario
	ucData[i++] = 0x00;   //14
	ucData[i++] = 0x00;   //15

	// 7. PHASORS
	for (int j = 0; j<10; j++){
		convert_float_to_char.pf = 0.1*j;//harmonics_T_mag[j];
		ucData[i++] = convert_float_to_char.byte[3];
		ucData[i++] = convert_float_to_char.byte[2];
		ucData[i++] = convert_float_to_char.byte[1];
		ucData[i++] = convert_float_to_char.byte[0];

		convert_float_to_char.pf = 0.0;//harmonics_T_phase[j];
		ucData[i++] = convert_float_to_char.byte[3];
		ucData[i++] = convert_float_to_char.byte[2];
		ucData[i++] = convert_float_to_char.byte[1];
		ucData[i++] = convert_float_to_char.byte[0];
	}

	// 8. FREQ = Desvio de frequencia do nominal, em mHz
	//Freq_final = (Freq_final - f0)*1000;

	convert_float_to_char.pf = 60.05;//(Freq_final);
	ucData[i++] = convert_float_to_char.byte[3];	//80
	ucData[i++] = convert_float_to_char.byte[2];	//81
	ucData[i++] = convert_float_to_char.byte[1];	//82
	ucData[i++] = convert_float_to_char.byte[0];	//83

	// 9. DFREQ = ROCOF in Hz/s "times 100"
	//ROCOF = ROCOF*100;

	convert_float_to_char.pf = 0.0;//(media_rocof*100);
	ucData[i++] = convert_float_to_char.byte[3];	//84
	ucData[i++] = convert_float_to_char.byte[2];	//85
	ucData[i++] = convert_float_to_char.byte[1];	//86
	ucData[i++] = convert_float_to_char.byte[0];	//87
#endif

	// 12. CRC-CCITT = Cyclic Redundancy Codes (CRC)
	CRC_CCITT = ComputeCRC(ucData, i);

	ucData[i++] = (unsigned char)((CRC_CCITT & 0xFF00) >> 8);	//88
	ucData[i++] = (unsigned char)(CRC_CCITT & 0x00FF);			//89

	return i;
}

uint16_t ComputeCRC(unsigned char *Message, uint16_t MessLen)
{
	uint16_t crc=0xFFFF;
	uint16_t temp;
	uint16_t quick;
	uint16_t i;

	for(i=0;i<MessLen;i++)
	{
		temp = (crc>>8) ^ Message[i];
		crc <<= 8;
		quick = temp ^ (temp >> 4);
		crc ^= quick;
		quick <<=5;
		crc ^= quick;
		quick <<= 7;
		crc ^= quick;
	}
	return crc;
}
