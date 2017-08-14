#ifndef ___DSAMECHANISMS___
#define ___DSAMECHANISMS___

#include <iostream>
#include <math.h>
#include <cmath>
#include <stdlib.h>
#include <stdio.h>
#include <ios>
#include <iomanip>
#include <unistd.h>
#include <ns3/object.h>



namespace ns3 {


class DsaMechanisms : public Object
{


	public:

		DsaMechanisms ();

		~DsaMechanisms ();

		static TypeId GetTypeId (void);

		void SetDsaMechanism (std::string dsa);

		std::string GetDsaMechanism () const;

		bool DFSGetDsaCondition (double Prradar);
		
		bool MainBeamGetDsaCondition (double Gmax, double Gnow, double Prradar, double Pthr);

		bool CoopGetDsaCondition (double Ptr, double Ptsec, double interf, double Prradar, double Ithr);

		void SetSUInterference (double su_interf);

		void SetGmax (double Gmax);

		void SetGain (double Gain);

		void SetNextBoolean (bool next);

		void SetMaxInterference (double maxinterf);


	private:

		bool m_opp, m_next;
		std::string m_dsa;
		double m_SUInterf, m_Gmax, m_Gain, m_maxSUInterf;

};

}

#endif
