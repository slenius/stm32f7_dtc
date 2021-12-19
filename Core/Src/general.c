
#include "general.h"

void GeneralInit(void){

	// https://community.st.com/s/question/0D50X0000AIdSc0SQF/unwanted-interrupts-for-ethernetmac-mmc
	ETH->MACIMR = ETH_MACIMR_TSTIM | ETH_MACIMR_PMTIM;
	ETH->MMCRIMR = ETH_MMCRIMR_RGUFM | ETH_MMCRIMR_RFAEM | ETH_MMCRIMR_RFCEM;
	ETH->MMCTIMR = ETH_MMCTIMR_TGFM | ETH_MMCTIMR_TGFMSCM | ETH_MMCTIMR_TGFSCM;
}

inline float lim_float(float v, float lim_lo, float lim_hi){
	if(v > lim_hi){
		return lim_hi;
	}
	if(v < lim_lo){
		return lim_lo;
	}
	return v;
}


