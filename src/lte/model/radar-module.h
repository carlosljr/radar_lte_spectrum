#ifndef ___RADAR___
#define ___RADAR___

#include <iostream>
#include <math.h>
#include <cmath>
#include <stdlib.h>
#include <stdio.h>
#include <ios>
#include <iomanip>
#include <unistd.h>
#include <ns3/object.h>
#include <ns3/nstime.h>
#include "ns3/vector.h"
#include "ns3/aggregateinterference.h"
#include <ns3/uinteger.h>
#include <ns3/mobility-model.h>
//Incluir as bibliotecas necessárias do ns-3.


namespace ns3 {


class radar : public Object
{


	double m_Ptx;
	uint16_t m_Angle_3dB;
	uint16_t m_dlEarfcn;
	uint16_t m_ulEarfcn;
	double m_Gmax;
	double m_Gsec;
	double m_Vang;
	double m_freq;
	double m_Ithr;
	double m_posx;
	double m_posy;
	double m_TXposx;
	double m_TXposy;
	double m_PU_band;
	double m_TxSU;
	bool m_intRadar;
	bool m_radarSpectrum;
	bool m_verify;
	Time m_txduration;
	double m_suInterference;
	Ptr <AggregateInterference> m_agg;
	Ptr <MobilityModel> m_mobility;

	public:

		static TypeId GetTypeId (void);

		radar ();

		~radar ();


		//This class gets the radar's angle reference based on the duration of the transmission. The area that radar is pointed to depends on its angular velocity.

		double get_reference (Time time);

		//Set SU's Tx Parameters for Interference calculation.

		void SetSUTxPower (double tx);

		//Set SU's Tx Duration;

		void SetTxDuration (Time tx_dur);

		void SetDlEarfcn (uint16_t dlEarfcn);

		uint16_t GetDlEarfcn () const;

		void SetUlEarfcn (uint16_t ulEarfcn);

		uint16_t GetUlEarfcn () const;

		//Set radar's paramaters.

		double GetTxPower () const;

		void SetTxPower (double ptx);

		uint16_t GetBeamwidth () const;

		void SetBeamwidth (uint16_t beam);

		double GetGmax () const;

		void SetGmax (double gmax);

		double GetGsec () const;

		void SetGsec (double gsec);

		double GetVang () const;

		void SetVang (double vang);

		double GetFreq () const;

		void SetFreq (double freq);

		double GetIthr () const;

		void SetIthr (double ithr);

		double GetBandwidth () const;

		void SetBandwidth (double band);		

		//Set node's position.

		void SetPositionRx (Vector pos);

		void VerPositionRx (Vector pos);

		//Set TX position.

		void SetPositionTx (Vector tx_pos);

		//Calculate the SU interference and return the value to the aggregate interference module.

		void CalculateSUInterference (Time t_now);

		//Remove interference from radar and remove from the aggregate interference class.

		void EndInterference ();

		//Instatiates AggregateInterference class.

		void SetAggInt (Ptr <AggregateInterference> agg);

		//Get the instantiated aggregate interference class.

		Ptr<AggregateInterference> GetAggregateInterference ();

		//Get the bool variable that tells if it's using the radar spectrum or not.

		bool GetSpectrumRadar ();

		//Set the bool variable that tells if it's using the radar spectrum or not.

		void SetSpectrumRadar (bool specRadar);

		double GetNodeAngle ();

		
		//It returns the amount of interference caused by the radar on the node referenced by its position. It considers transition points if exists during the transmission time.

		double get_interference (Time tnow, Time tant);

		//It calculates the pathloss of radar's signal based on Logdistance Propagation Model and the effect earth's curvature.

		double pathloss (double Gain);

		//This class returns the gain of radar's antenna for a given a node.

		double get_gain (double noderef, double radar_reference);
		
		//It transforms (x,y) to a reference angle.

		double get_nodeangle (double x, double y);


		//Reference gain of radar's antenna for a given node.

		double calc_gain (double radar_angle, double node_angle);

		//Set Radar's MobilityModel.

                void SetMobility (Ptr<MobilityModel> mm);

                //Get Radar's MobilityModel.

                Ptr<MobilityModel> GetMobility ();


		

};

}

#endif
