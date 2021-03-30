/*
 * Hajoitus_ver3.c
 *
 * Created: 7.5.2019 17:19:57
 * Author : s708153
 */ 

#define F_CPU 16000000UL		// m‰‰ritet‰‰n prosessorin taajuus
#include <avr/io.h>				//lis‰t‰‰n kirjastoja
#include <util/delay.h>			// ------||--------
#include <avr/interrupt.h>		//-------||--------
#include <stdlib.h>
//-------------------------------moottorin suunta-------------------------------------------------
typedef enum {
	M_CW  = 1,				// myˆt‰p‰iv‰‰n
	M_CCW = -1				//vastap‰iv‰‰n
} M_DIR;

// Timer0 ja Timer1 ohjausarvot
typedef enum {
	TIMER_INIT  = 0,				// case 0 timerin alustus
	TIMER_START = 1,				// case 1 timerin starttaus
	TIMER_STOP  = 2				// case 2 timerin lopetus
}TIMER_STATE;

//--------------------------------MƒƒRITTELYT--------------------------------------------------
#define TIMER1_100MS_CNT	(6249)		// vertailuarvo Timer1 varten/ vertailu t‰sm‰‰ kun
//6250 * 1/(16000000/256) = 100ms
#define MAX_STEPS       (600)			// steppej‰ per pyˆr‰hdys
#define STEP_PULSE_LEN  (24)			// yhden pulsin kesto
#define MAX_STEP_SEQ	(3)				// steppien m‰‰r‰ step sequense -1
#define MinFreq			(30)			// minimitaajuus
#define MaxFreq			(200)			// Maksimitaajuus
#define K				(3306UL)		//
#define MinStep			(5)				// Minimiaskeleet
#define ADC_LIMITS		(38)			//

const uint8_t aStepSequence[] = {

	// Full step - two phases on -drive sequence
	(_BV(PD4) | _BV(PD6)),	// A+ & B+
	(_BV(PD4) | _BV(PD7)),	// A+ & B-
	(_BV(PD5) | _BV(PD7)),	// A- & B-
	(_BV(PD5) | _BV(PD6))	// A- & B+
};

//--------------------------------MUUTTUJAT----------------------------------------------------
volatile uint16_t guPulseCounter = 0;	// pulssien lukum‰‰r‰/100ms
volatile uint16_t gu100msPulses  = 0;	// lasketaan 100ms pulssit
volatile uint8_t  gustepSeqInd  = 0;	// ajosekvenssin indeksi
volatile M_DIR	 direction   = M_CW;	// moottorin suunta
volatile uint16_t gustepCounter = 0;	// askelten laskemiseen
volatile int16_t giNextStepPos = 0;		// viisarin liikutus seuraavaan
volatile uint16_t guCurStepPos = 0;		// viisarin liikutusaskel
volatile int16_t giDeltaStepPos = 0;	// askelminen v‰linen muutos
volatile uint16_t guOffset = 19;

//-------------------------------FUNKTIO ESITTELYT---------------------------------------------
uint16_t readADC();						// Potenttiometrin luku
void Timer0(TIMER_STATE state);			// Timer0
void Timer1(TIMER_STATE state);			// Timer1
void TakeSteps(int16_t usHowManySteps);	// Askeleet, positiiviset myˆt‰p‰iv‰‰n ja negatiiviset vastap‰iv‰‰n

//-------------------------------MAIN-----------------------------------------------------------
int main(void)
{
	uint16_t uChangePosition;
	uint8_t ucLimitInd;
	
	uint16_t auLimits[ADC_LIMITS] = {
		1003,	// [0] >4,78V arvot saatu kaavalla 1003*(4,88V/1024)=4,74V, 4,88 saatu mittaamalla
		983,	// [1] >
		963,	// [2] >
		943,	// [3] >
		923,	// [4] >
		903,	// [5] >
		883,	// [6] >
		863,	// [7] >
		843,	// [8] >
		823,	// [9] >
		803,	// [10] >
		783,	// [11] >
		763,	// [12] >
		743,	// [13] >
		723,	// [14] >
		703,	// [15] >
		683,	// [16] >
		663,	// [17] >
		643,	// [18] >3,06V ------------------yli 3V-----------------
		418,	// [19] >1,99V ------------------alle 2V----------------
		396,	// [20] >
		374,	// [21] >
		352,	// [22] >
		330,	// [23] >
		308,	// [24] >
		286,	// [25] >
		264,	// [26] >
		242,	// [27] >
		220,	// [28] >
		198,	// [29] >
		176,	// [30] >
		154,	// [31] >
		132,	// [32] >
		110,	// [33] >
		88,		// [34] >
		66,		// [35] >
		44,		// [36] >
		22		// [37] >
	};
	
	DDRD = (_BV(PD5) | _BV(PD4) | _BV(PD6) | _BV(PD7));		//pinnit PD4-PD7 asetetaan outputiksi DDRD
	DDRB &= ~(_BV(PB0));									//PB(0) inputiksi DDRB rekisteriin
	
	sei();													// keskeytykset p‰‰lle
	Timer0(TIMER_INIT);										// alustetaan timer0
	Timer1(TIMER_INIT);										// alustetaan timer1
	Timer1(TIMER_START);									// Timer1 pyˆrim‰‰n taustalla
	
	TakeSteps(-MAX_STEPS-1);								// siirret‰‰n viisari nollaan
	
	ADMUX |= (_BV(MUX1) | _BV(MUX0));						// ADC3 (UNO #A3) valitaan k‰yttˆˆn
	ADMUX |= _BV(REFS0);									// referenssij‰nnite 5V
	ADCSRA |= (_BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0));		// valitaan jakajaksi 128
	ADCSRA &= ~(_BV(ADATE));								// otetaan auto trigger pois
	ADCSRB &= ~(_BV(ADTS2) | _BV(ADTS1) | _BV(ADTS0));		// valitaan free running
	
	while(1){
		uChangePosition = readADC(ADCH3);					// Luetaan montako askelta oteaan
		
		for (ucLimitInd = 0; ucLimitInd < ADC_LIMITS; ucLimitInd++) { // kasvatetaan 
			if(uChangePosition < auLimits[ucLimitInd])break;
		}
		if (gu100msPulses < MinFreq) {						// JOs ms pulssienm‰‰r‰ on alle 30 ajetaan nollapositioon
			giNextStepPos = 0;
		}
		else{
			giNextStepPos = K*(gu100msPulses-30UL)/1000UL+guOffset; // muulloin lasketaan sueraava positio ja lis‰t‰‰n offset
		}
		if (giNextStepPos > MAX_STEPS) {					// jos suurempi kuin max, ajetaan maksimipositioon 
			giNextStepPos = MAX_STEPS;						
		}
		else if(giNextStepPos < 0) {						// jos tuleva positio < 0, ajetaan nollaan
			giNextStepPos = 0;
		}
		
		giDeltaStepPos = giNextStepPos -guCurStepPos;		// haluttu muutos
		
		
		if (abs(giDeltaStepPos) > MinStep) {				// jos muutoksen suuruss yli minstep, toteutetaan takesteps
			TakeSteps(giDeltaStepPos);						// otetaan askeleet
			guCurStepPos = guCurStepPos + giDeltaStepPos;	// p‰ivitett‰‰n nykyinwn positio
		}
		
	}//--------------------------While p‰‰ttyy----------------------------------------------
	
}//------------------------------main p‰‰ttyy-----------------------------------------------

//---------------------------------Hienos‰‰tˆ potenttiometrill‰---------------------------
uint16_t readADC()
{
	uint16_t uADCval;
	
	ADCSRA |= _BV(ADEN);
	ADCSRA |= _BV(ADSC);
	
	while (ADCSRA & _BV(ADSC));
	
	uADCval = ADC;
	ADCSRA &= ~(_BV(ADEN));
	
	return uADCval;
}//readADC p‰‰ttyy

//-------------------------------Funktio takeSteps------------------------------------------
void TakeSteps(int16_t usHowManySteps)
// --------------------------------------
{

	if (usHowManySteps > 0) {					// Funktion saadessa positiivisen
		direction = M_CW;						// moottori pyˆrii vastap‰iv‰‰n
	}
	else {
		direction = M_CCW;						// negatiivisella vastap‰iv‰‰n
		usHowManySteps = -usHowManySteps;		// askeleet muutetaan positiivisiksi
	}
	gustepCounter = 0;							// nollataan askellaskuri

	Timer0(TIMER_START);						// Timer0 k‰yntiin
	while (!(gustepCounter == usHowManySteps)) {//tyhj‰ silmukka kunnes askeleet tehty
		;
	}
	Timer0(TIMER_STOP);							// Pys‰ytet‰‰n Timer0
} // ---------------------------takeSteps p‰‰ttyy-----------------------------------------


// ------------------------------Funktio Timer1-----------------------------------------
void Timer1(TIMER_STATE state)
{
	switch (state) {
		
		case TIMER_INIT:							// Timerin tilan asetus
		TCCR1B |= (_BV(WGM12));						// CTC mode ( mode 4)
		TCCR1B &= ~(_BV(ICES1));					//nappaus pulssin tippuvalla reunalla
		break;
		
		case TIMER_START:							//Timerin aloitus
		OCR1A = TIMER1_100MS_CNT;					// 100ms rekisteriin
		TCNT1 = 0x00;								// laskurin nollaus
		TIFR1 |= (_BV(ICF1) | _BV(OCF1A));			// tyhjennet‰‰n IQR varmuudeksi
		TIMSK1 |= (_BV(ICIE1) | _BV(OCIE1A));		// input p‰‰lle ja keskeytykset
		TCCR1B |= (_BV(ICES1));					// k‰ynnistet‰‰n timer1
		break;
		
		case TIMER_STOP:							//timerin pys‰ytys
		default:
		TCCR1B &= ~(_BV(CS12));						// pys‰ytet‰‰n timer1
		TIMSK1 &= ~(_BV(ICIE1) | _BV(OCIE1A));		//input pois ja keskeytykset
		break;
		}
	}//---------------------------------Timer1 p‰‰ttyy--------------------------------------

	// --------------------------------Funktio Timer0-------------------------------------------
	void Timer0(TIMER_STATE state)
	{
		
		switch (state) {
			case TIMER_INIT:						// Timerin tilan asetus
			TCCR0A |= (_BV(WGM01));					// CTC mode (Mode 2)
			break;
			
			case TIMER_START:						//Timerin aloitus
			OCR0A = STEP_PULSE_LEN;					//pulssin pituus vied‰‰n rekisteriin
			TCNT0 = 0x00;							// laskurin nollaus
			TIFR0  |= _BV(OCIE0A);					// tyhjennet‰‰n IQR varmuudeksi
			TIMSK0 |= (_BV(OCIE0A));				// Timer0:lle asetetaan keskeytykset, koska askellus tehd‰‰n ISR(TIMER0_COMPA_vect)
			TCCR0B |= (_BV(CS00) | _BV(CS02));		// jakaja 1024, k‰ynnistet‰‰n timer0
			break;
			
			case TIMER_STOP:						//timerin pys‰ytys
			default:
			TCCR0B &= ~(_BV(CS00) | _BV(CS02));		// pys‰ytet‰‰n timer0
			TIMSK0 &= ~(_BV(OCIE0A));				// keskeytykset pois
			break;
		}
		
	} //-------------------------------Timer0 p‰‰ttyy-------------------------------------------

	ISR(TIMER1_COMPA_vect)// ----------Taajuuden m‰‰ritys keskeytysk‰sittelij‰ss‰---------------
	{
		gu100msPulses  = guPulseCounter;			//
		guPulseCounter = 0;							// nollataan laskuri seuraavaa 100ms varten
	}//--------------------------------ISR p‰‰ttyy----------------------------------------------

	ISR(TIMER1_CAPT_vect)//------------Pulssien laskevien reunojen laskenta---------------------
	{
		ICR1 = 0x00;								// nollataan kaappausrekisteri
		guPulseCounter++;							// kasvatetaan pulssilaskuria
	}//--------------------------------ISR p‰‰ttyy---------------------------------------------

	ISR(TIMER0_COMPA_vect)//-----------Keskeytysk‰sittelij‰: 1,6ms askeleet--------------------
	{
		PORTD = aStepSequence[gustepSeqInd];		// portti D:hen kirjoitetaan ajosekvenssi
		gustepSeqInd += direction;					// seuraava askel oikeaan suuntaan
		gustepSeqInd &= MAX_STEP_SEQ;				// pidet‰‰n sekvenssi 0-3
		gustepCounter++;							// lasketaan askelia
	}
