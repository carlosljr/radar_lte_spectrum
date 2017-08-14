//#include <iostream>
//#include <math.h>
//#include <stdlib.h>
//#include <ios>
//#include <iomanip>
//#include <unistd.h>
#include "ns3/radar-module.h"
#include <ns3/object-factory.h>
#include <ns3/double.h>
#include <ns3/attribute-accessor-helper.h>
//#include <ns3/uinteger.h>
#include <ns3/simulator.h>
//Adicionar bibliotecas necessárias do ns-3.
//Modifcar parâmetros e tipos do ns-3.


namespace ns3 {


TypeId radar::GetTypeId (void)
{

  //std::cout << "Chamou!" << std::endl;

  static TypeId
    tid =
    TypeId ("ns3::radar")
    .SetParent<Object> ()
    .AddConstructor<radar> ()
    .AddAttribute ("RadarTxPower",
                   "Radar's transmission power [dBm]",
                   DoubleValue (63.5),
                   MakeDoubleAccessor (&radar::SetTxPower,
				       &radar::GetTxPower),
                   MakeDoubleChecker<double> ())

    .AddAttribute ("Angle3dB",
                   "Radar's beamwidth [degrees]",
                   UintegerValue (12),
                   MakeUintegerAccessor (&radar::SetBeamwidth,
					 &radar::GetBeamwidth),
                   MakeUintegerChecker<uint16_t> ())

    .AddAttribute ("DlEarfcn",
                   "Radar's Downlink Earfcn",
                   UintegerValue (39650),
                   MakeUintegerAccessor (&radar::m_dlEarfcn),
                   MakeUintegerChecker<uint16_t> (0, 39650))

     .AddAttribute ("UlEarfcn",
                   "Radar's Uplink Earfcn",
                   UintegerValue (39651),
                   MakeUintegerAccessor (&radar::m_ulEarfcn),
                   MakeUintegerChecker<uint16_t> (0, 39651))

    .AddAttribute ("Gmax",
                   "Radar's Primary Lobe Gain [dBi]",
                   DoubleValue (44),
                   MakeDoubleAccessor (&radar::SetGmax,
				       &radar::GetGmax),
                   MakeDoubleChecker<double> ())

    .AddAttribute ("Gsec",
                   "Radar's Secondary Lobe Gain [dBi]",
                   DoubleValue (-21),
                   MakeDoubleAccessor (&radar::SetGsec,
				       &radar::GetGsec),
                   MakeDoubleChecker<double> ())

    .AddAttribute ("Vang",
                   "Radar's angular velocity [degrees/s]",
                   DoubleValue (1.2),
                   MakeDoubleAccessor (&radar::SetVang,
				       &radar::GetVang),
                   MakeDoubleChecker<double> ())

    .AddAttribute ("Frequency",
                   "Radar's operation frequency [GHz]",
                   DoubleValue (5.6),
                   MakeDoubleAccessor (&radar::SetFreq,
				       &radar::GetFreq),
                   MakeDoubleChecker<double> ())

    .AddAttribute ("Ithr",
                   "Radar's interference threshold [dB]",
                   DoubleValue (-107),
                   MakeDoubleAccessor (&radar::SetIthr,
				       &radar::GetIthr),
                   MakeDoubleChecker<double> ())

    .AddAttribute ("RadarBand",
                   "Radar's bandwidth [MHz]",
                   DoubleValue (10),
                   MakeDoubleAccessor (&radar::SetBandwidth,
				       &radar::GetBandwidth),
                   MakeDoubleChecker<double> ())
   ;

   return tid;

}


radar::radar () {m_radarSpectrum = false;}

radar::~radar () {}


void
radar::SetMobility (Ptr<MobilityModel> mm) {

        m_mobility = mm;

}

Ptr<MobilityModel>
radar::GetMobility () {

        return m_mobility;

}

double
radar::get_reference (Time time) { 


	 double ref_angle = m_Vang * time.GetSeconds ();

	if (ref_angle >= 360.0) {

		ref_angle = ref_angle - 360.0;

	}

	//fprintf (stdout, "Radar angle alternative %3.6f \n", ref_angle);

	//std::cout <<"Radar angle " << ref_angle << std::endl;

	return ref_angle;

}

void
radar::SetDlEarfcn (uint16_t dlEarfcn) {

  m_dlEarfcn = dlEarfcn;

}

uint16_t
radar::GetDlEarfcn () const {

  return m_dlEarfcn;

}

void
radar::SetUlEarfcn (uint16_t ulEarfcn) {

  m_ulEarfcn = ulEarfcn;

}

uint16_t
radar::GetUlEarfcn () const{

  return m_ulEarfcn;

}

void
radar::SetSUTxPower (double tx) {

  m_TxSU = tx;

}

void
radar::SetTxDuration (Time tx_dur) {

 m_txduration = tx_dur;

}


double
radar::GetTxPower () const {

	return m_Ptx;
}


void
radar::SetTxPower (double ptx) {

	m_Ptx = ptx;
	
}

uint16_t
radar::GetBeamwidth () const {

	return m_Angle_3dB;

}

void
radar::SetBeamwidth (uint16_t beam) {

	m_Angle_3dB = beam;

}

double
radar::GetGmax () const {

	return m_Gmax;
}

		
void
radar::SetGmax (double gmax) {

	m_Gmax = gmax;

}

double
radar::GetGsec () const {

	return m_Gsec;

}


void
radar::SetGsec (double gsec) {

	m_Gsec = gsec;

}

double
radar::GetVang () const {

	return m_Vang;

}


void
radar::SetVang (double vang) {

	m_Vang = vang;

}

double
radar::GetFreq () const {

	return m_freq;

}

void
radar::SetFreq (double freq) {

	m_freq = freq;

}

double
radar::GetIthr () const {

	return m_Ithr;
}

void
radar::SetIthr (double ithr) {

	m_Ithr = ithr;

}

double
radar::GetBandwidth () const {

	return m_PU_band;

}

void
radar::SetBandwidth (double band) {

	m_PU_band = band;

}


void
radar::SetPositionRx (Vector pos) {

	m_posx = pos.x;
	m_posy = pos.y;
	m_intRadar = false;
	m_verify = false;

	//std::cout << "RX Pos x = " << m_posx << std::endl
		  //<< "RX Pos y = " << m_posy << std::endl;

}

void
radar::VerPositionRx (Vector pos) {

	m_posx = pos.x;
	m_posy = pos.y;
	m_intRadar = false;
	m_verify = true;

	//std::cout //<< "RX Pos x = " << m_posx << std::endl
		  //<< "RX Pos y = " << m_posy << std::endl;

}

void
radar::SetPositionTx (Vector tx_pos) {

	m_TXposx = tx_pos.x;
	m_TXposy = tx_pos.y;
	m_intRadar = true;

	//std::cout << "TX Pos X " << m_TXposx << std::endl
		  //<< "TX Pos Y " << m_TXposy << std::endl;
}

void
radar::CalculateSUInterference (Time t_now) {

	Time t_fut = t_now + m_txduration;

	double angle_now = get_reference (t_now);

	double angle_fut = get_reference (t_fut);

	double nodeangle = get_nodeangle (m_TXposx, m_TXposy);

	double Gfut = get_gain (nodeangle, angle_fut);
	double Gnow = get_gain (nodeangle, angle_now);

	if (Gfut != Gnow) {

		m_suInterference = pathloss (Gfut);

	} else {

		m_suInterference = pathloss (Gnow);

	}

	m_agg->AddInterference (m_suInterference);

	//std::cout << "SU Interference: " << m_suInterference << std::endl;


}

void
radar::EndInterference () {

	m_agg->RemoveInterference (m_suInterference);
	m_suInterference = 0.0;
}

double
radar::GetNodeAngle () {

	return get_nodeangle (m_posx, m_posy);

}
	

double
radar::get_interference (Time tnow, Time tant) { // variáveis vindas do ns-3	

	double angle_now = get_reference (tnow);

	double angle_ant = get_reference (tant);

	Time duration = tnow - tant;

	//double Gant = get_gain (angle_ant);

	//double Gnow = get_gain (angle_now);

	double nodeangle;

	if (!m_intRadar) {

		nodeangle = get_nodeangle (m_posx, m_posy);
		//std::cout << "SU receiving interference..." << std::endl;
		//Gant = get_gain (angle_ant);
		//Gnow = get_gain (angle_now);

	} else {

		nodeangle = get_nodeangle (m_TXposx, m_TXposy);
		//std::cout << "Radar receiving interference..." << std::endl;
		//Gant = get_gain (angle_ant);
		//Gnow = get_gain (angle_now);

	}

	double Gant = get_gain (nodeangle, angle_ant);
	double Gnow = get_gain (nodeangle, angle_now);

	double angle_radar_sec;

	double interf;

	//std::cout << "Ptx: " <<	m_Ptx << std::endl
		  //<< "Angle: " << m_Angle_3dB << std::endl
		  //<< "Gmax: " << m_Gmax << std::endl
	 	  //<< "Gsec: " << m_Gsec << std::endl
		  //<< "Vang: " << m_Vang << std::endl
		  //<< "Freq: " << m_freq << std::endl
		  //<< "Ithr: " << m_Ithr << std::endl
		  //<< "Bandwidth: " << m_PU_band << std::endl;
 

	//fprintf (stdout, "Angle alternative before %3.6f \n", angle_ant);
	//fprintf (stdout, "Angle alternative now %3.6f \n", angle_now);

	//std::cout << "Reference angle before " << angle_ant << std::endl
                  //<< "Reference angle now " << angle_now << std::endl
		  //<< "Node reference " << nodeangle << std::endl
		  //<< "Gant " << Gant << std::endl
		  //<< "Gnow " << Gnow << std::endl
		  //<< "Duration of Interference " << duration.GetSeconds () << std::endl;

	if (Gant != Gnow) {

		if (Gant < Gnow) {

			if (nodeangle == 0.0) {

				angle_radar_sec = 360 - (angle_ant + (m_Angle_3dB/2.0));

			} else {

				angle_radar_sec = (360 - (nodeangle + angle_ant)) - (m_Angle_3dB/2.0);

			}


		} else {

			if (nodeangle == 0.0) {

				angle_radar_sec = (m_Angle_3dB/2.0) - angle_ant;

			} else {

				angle_radar_sec = (m_Angle_3dB/2.0) - (nodeangle + angle_ant - 360);

			}

		}

		double t1 = angle_radar_sec / m_Vang;

		double interf1 = pathloss (Gant);

		double interf2 = pathloss (Gnow);

		//std::cout << "Duration for Gant: " << t1 << std::endl
			  //<< "Interference 1: " << interf1 << std::endl
		          //<< "Interference 2: " << interf2 << std::endl;

		//double interf1 = 12.0 * Gant * t1;

		//double interf2 = 12.0 * Gnow * (m_duration - t1);

		interf = ((interf1*t1) + (interf2*(duration.GetSeconds () - t1))) / duration.GetSeconds ();

	} else {

		interf = pathloss (Gnow);

	}

	//std::cout << "Interference result: " << interf << std::endl;

	return interf;
}

double
radar::pathloss (double Gain) {

	double d, dist, f, L0, d_km, L, Gsec, Prx, Prx_w;

	if (!m_intRadar) {

		if (!m_verify) {

			d = sqrt (pow(m_posx, 2) + pow(m_posy, 2));

			dist = 20*log10(1.0); //Distancia de referencia, em Km.

			f = 20*log10(m_freq); //Frequencia, em GHz.

			L0 = 92.44 + dist + f;

			d_km = d * pow(10.0, -3.0);

			L = L0 + 10.0*3.0*log10 (d_km/1.0); //Logdistance - Fator gamma = 3.5

			Gsec = 0.0; //Ganho considerado para a antena isotrópica. Válido tanto para UE quanto eNB

			//double SU_band = 5.0; //HARD CODE: Largura de Banda do LTE para 25 RBs.

			//double cp;

			//if (SU_band/PU_band > 1.0) {

			//cp = 1.0;

			//} else {

				//cp = SU_band/PU_band; //Fator de acoplamento entre largura de banda do radar e largura de banda do SU.

			//}

			//double cp_dB = 10 * log10 (cp); //Fator de acoplamento, em dB.

			Prx = m_Ptx + Gain + Gsec - L; //Prx, em dBm.

			Prx_w = pow(10.0, Prx/10.0) * pow(10.0, -3.0); //Prx, em W.

			double Psd_RX = (Prx_w / (m_PU_band * pow(10.0, 6.0))); //Densidade espectral de Potência recebida (W/Hz) dentro da banda de radar.

			double Psd_RB = Psd_RX * ((180000 * 15) / (m_PU_band * pow(10.0, 6.0))); //Interferência resultante para um RB. Porcentagem da PSD que interfere no RB.

			////std::cout << "Distancia " << d << std::endl
		  	  	//<< "Distancia em Km " << d_km << std::endl
		  	  	//<< "Valor de L0 " << L0 << std::endl
		  	  	//<< "Valor de L " << L << std::endl
		  	  	//<< "Potencia recebida [dBm] " << Prx << std::endl
		  	  	//<< "Potencia recebida [W] " << Prx_w << std::endl
		  	  	//<< "Psd RX: " << Psd_RX << std::endl;

			//std::cout << "Valor de interferência no SU: " << Psd_RB << std::endl;

			return Psd_RB;

		} else {

			d = sqrt (pow(m_posx, 2) + pow(m_posy, 2));

			dist = 20*log10(1.0); //Distancia de referencia, em Km.

			f = 20*log10(m_freq); //Frequencia, em GHz.

			L0 = 92.44 + dist + f;

			d_km = d * pow(10.0, -3.0);

			L = L0 + 10.0*3.0*log10 (d_km/1.0); //Logdistance - Fator gamma = 3.5

			Gsec = 0.0; //Ganho considerado para a antena isotrópica. Válido tanto para UE quanto eNB

			double SU_band = 3.0; //HARD CODE: Largura de Banda do LTE para 15 RBs.

			double cp;

			if (SU_band/m_PU_band > 1.0) {

				cp = 1.0;

			} else {

				cp = SU_band/m_PU_band; //Fator de acoplamento entre largura de banda do radar e largura de banda do SU.

			}

			double cp_dB = 10 * log10 (cp); //Fator de acoplamento, em dB.

			Prx = m_Ptx + Gain + Gsec - L + cp_dB; //Prx, em dBm.

			//std::cout << "Sensing: Radar's interference, in dBm." << std::endl;

			return Prx;

	      }



	} else {

		d = sqrt (pow(m_TXposx, 2) + pow(m_TXposy, 2));

		dist = 20*log10(1.0); //Distancia de referencia, em Km.
		
		double freq;
		
		if (m_TxSU >= 20) {

			freq = 5.6e9 + (2*(1.5e6));

			//std::cout << "Interferência de transmissão do downlink." << std::cout;

		} else {

			freq = 5.6e9 - (2*(1.5e6));
			
			//std::cout << "Interferência de transmissão do uplink." << std::cout;

		}

		double f_ghz = freq / pow (10.0, 9.0);

		//std::cout << "Freq em GHz: " << f_ghz << std::endl
			  //<< "Freq: " << freq << std::endl;

		f = 20*log10(f_ghz); //Frequencia, em GHz.

		L0 = 92.44 + dist + f;

		d_km = d * pow(10.0, -3.0);

		L = L0 + 10.0*3.0*log10 (d_km/1.0); //Logdistance - Fator gamma = 3.5

		Gsec = 0.0; //Ganho considerado para a antena isotrópica. Válido tanto para UE quanto eNB

		Prx = m_TxSU + Gain + Gsec - L;

		Prx_w = pow(10.0, Prx/10.0);

			//std::cout << "Distancia " << d << std::endl
		  	  	//<< "Distancia em Km " << d_km << std::endl
				//<< "Potência de transmissão: " << m_TxSU << std::endl 
		  	  	//<< "Valor de L0 " << L0 << std::endl
		  	 	//<< "Valor de L " << L << std::endl
		  	  	//<< "Potencia recebida [dBm] " << Prx << std::endl
		  	  	//<< "Potencia recebida [W] " << Prx_w << std::endl;



		//std::cout << "Valor de interferência no Radar: " << Prx_w << std::endl;

		return Prx_w;

	}

		//std::cout << "Distancia " << d << std::endl
		//  	  << "Distancia em Km " << d_km << std::endl
		//  	  << "Valor de L0 " << L0 << std::endl
		//  	  << "Valor de L " << L << std::endl
		//  	  << "Potencia recebida [dBm] " << Prx << std::endl
		//  	  << "Potencia recebida [W] " << Prx_w << std::endl;





}

double
radar::get_gain (double noderef, double radar_reference) {

	double G = calc_gain (radar_reference, noderef);

	//std::cout << "Gain pointed to the node: " << G << std::endl;

	return G;

}

#define PI 3.14159265

double 
radar::get_nodeangle (double x, double y) {

	double angle;

	if (x >= 0 && y > 0) {

		if (x == 0) {

			angle = 90.0;

		} else {

			angle = (atan(y/x) * 180.0) / PI;

		}

	} else if (x > 0 && y == 0) {

		angle = 0.0;

	} else if (x < 0 && y >= 0) {

		if (y == 0) {

			angle = 180.0;

		} else {

			double xpos = x * -1;

			angle = 180 - ((atan(y/xpos) * 180.0) / PI);

		}


	} else if (x <= 0 && y < 0) {

		if (x == 0) {

			angle = 270.0;

		} else {

			double ypos = y * -1;

			double xpos = x * -1;

			angle = 180 + ((atan(ypos/xpos) * 180.0) / PI);

		}

	} else if (x > 0 && y < 0) {

		double ypos = y * -1;

		angle = 360 - ((atan(ypos/x) * 180.0) / PI);

	} else {

		angle = 0.0;

	}

	//std::cout << "Secondary angle " << angle << std::endl;

	return angle;

}


double 
radar::calc_gain (double radar_angle, double node_angle) {

	double G;

	//if (radar_angle + node_angle == 360 || radar_angle + node_angle == 0) {

		//G = m_Gmax;

	//} else 

	if (radar_angle + node_angle >= 360 || radar_angle + node_angle == 0) {

		if (radar_angle + node_angle - 360 <= m_Angle_3dB/2 || 720 - (radar_angle + node_angle) <= m_Angle_3dB/2) {

			G = m_Gmax;

		} else {

			G = m_Gsec;

		}

	} else {

		if (radar_angle + node_angle <= m_Angle_3dB/2 || radar_angle + node_angle + (m_Angle_3dB/2) >= 360) {

			G = m_Gmax;

		} else {

			G = m_Gsec;

		}


	}

	return G;

}

//Novo método: Instancia objeto AggregateInterference.

void
radar::SetAggInt (Ptr<AggregateInterference> agg) {

	m_agg = agg;

}

bool
radar::GetSpectrumRadar () {

	return m_radarSpectrum;

}

void
radar::SetSpectrumRadar (bool specRadar) {

	m_radarSpectrum = specRadar;

}

Ptr<AggregateInterference>
radar::GetAggregateInterference () {

	return m_agg;

}

}
