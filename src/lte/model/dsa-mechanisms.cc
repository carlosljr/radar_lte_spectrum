#include <ns3/object-factory.h>
#include <ns3/double.h>
#include <ns3/uinteger.h>
#include <ns3/simulator.h>
#include <ns3/string.h>
#include "ns3/dsa-mechanisms.h"
#include <ns3/attribute-accessor-helper.h>
#include <ns3/trace-source-accessor.h>


namespace ns3 {


DsaMechanisms::DsaMechanisms () {m_opp = false; }

DsaMechanisms::~DsaMechanisms () {}

TypeId DsaMechanisms::GetTypeId (void)
{

  static TypeId
    tid =
    TypeId ("ns3::DsaMechanisms")
    .SetParent<Object> ()
    .AddConstructor<DsaMechanisms> ()
    .AddAttribute ("DsaMechanism",
                   "Set the type of DSA mechanism to use.",
                   StringValue ("NoRadar"),
                   MakeStringAccessor (&DsaMechanisms::SetDsaMechanism,
                                       &DsaMechanisms::GetDsaMechanism),
                   MakeStringChecker ())

   ;

   return tid;
}


void
DsaMechanisms::SetDsaMechanism (std::string dsa) {

	m_dsa = dsa;

}

std::string
DsaMechanisms::GetDsaMechanism () const {

	return m_dsa;

}

bool
DsaMechanisms::DFSGetDsaCondition (double Prradar) {

	double threshold = -64.0;

	if (threshold > Prradar) {

		m_opp = true;

	} else {

		m_opp = false;

	}

	return m_opp;


}

bool
DsaMechanisms::MainBeamGetDsaCondition (double Gmax, double Gnow, double Prradar, double Pthr) {

	if ((Pthr > Prradar) && (Gnow != Gmax)) {

		m_opp = true;

	} else {

		m_opp = false;

	}

	return m_opp;

}

bool
DsaMechanisms::CoopGetDsaCondition (double Ptr, double Ptsec, double interf, double Prradar, double Ithr) {

	//Converting all the transmitted power and the interference threshold to mW.

	double Ptr_mw = pow (10.0, Ptr/10.0);

	double Ptsec_mw = pow (10.0, Ptsec/10.0);

	double Ithr_mw = pow (10.0, Ithr/10.0);

	//Calculating dinamically the threshold to be used as reference.

	double Pthr_mw = (Ithr_mw - interf) * Ptr_mw / Ptsec_mw;

	//Converting Threshold to dBm for comparision.

	double Pthr = 10*log10(Pthr_mw);

	//std::cout << "interf: " << interf << std::endl
		  //<< "Threshold: " << Pthr << std::endl
		  //<< "Thr in mW: " << Pthr_mw << std::endl
		  //<< "Ithr in mW: " << Ithr_mw << std::endl;

	if (Pthr > Prradar) {

		double new_interf = interf + m_SUInterf; //Atualizando interferência agregada e verificar se ultrapassa o threshold.

		if (Ithr_mw > new_interf) { //Se não ultrapassa...

			if (m_Gain == m_Gmax) { //Verificar se US já está na direcao do lobulo principal. Se estiver, então US está na zona 3. Transmite o tempo todo. INSERIR VARIÁVEL GAIN PARA A VARIAVEL DO GANHO ATUAL. INSERIR VARIAVEL GMAX QUE GUARDA O VALOR DO GMAX DO RADAR.

		//std::cout << "Thr > interf" << std::endl;

				m_opp = true;

			} else { //Senão, US pode estar na zona 3 ou 2. Caso seja zona 2, verificar se no maior tempo de transmissao ate o proximo sensoreamento o lobulo estara apontado para o radar em algum instante.

				if (m_next) { // Booleano que diz se o lobulo estara ou nao apontado para o US. INSERIR MNEXT.

					double max_interf = interf + m_maxSUInterf; //INSERIR MAXSUINTERF.

					if (Ithr_mw > max_interf) { //Se interferencia for menor que o threshold, US está na zona 3. Portanto, pode transmitir.

						m_opp = true;

					} else { //Senão, US está na zona 2, e não deve transmitir, pois durante o período de transmissão o US irá interferir acima do threshold.

						m_opp = false;

					}

				} else { //Caso o lobulo principal não venha a estar apontado para o US, transmite na banda de radar.


						m_opp = true;


				}

			}

		} else { //Não pode transmitir, pois a nova interferência proporciona uma interferencia acima do threshold.

			m_opp = false;

		}

	} else { //Não transmite, pois Prradar já está acima do threshold.

		//std::cout << "Thr < interf" << std::endl;

		m_opp = false;

	}

	return m_opp;
	
}

void
DsaMechanisms::SetSUInterference (double su_interf) {

	m_SUInterf = su_interf;

}


void
DsaMechanisms::SetGmax (double Gmax) {

	m_Gmax = Gmax;

}

void
DsaMechanisms::SetGain (double Gain) {

	m_Gain = Gain;

}

void
DsaMechanisms::SetNextBoolean (bool next) {

	m_next = next;

}

void
DsaMechanisms::SetMaxInterference (double maxinterf) {

	m_maxSUInterf = maxinterf;

}
	

} 
