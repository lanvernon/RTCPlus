//
//                           SimuLTE
//
// This file is part of a software released under the license included in file
// "license.pdf". This license can be also found at http://www.ltesimulator.com/
// The above file and the present reference are part of the software itself,
// and cannot be removed from it.
//

#ifndef _LTE_LTECHANNELMODEL_H_
#define _LTE_LTECHANNELMODEL_H_
#include "common/LteCommon.h"
#include "common/LteControlInfo.h"

class LteAirFrame;

class LteChannelModel
{
  protected:
    unsigned int band_;
    public:
    LteChannelModel(unsigned int band);
    virtual ~LteChannelModel();
    /*
     * Compute the error probability of the transmitted packet according to cqi used, txmode, and the received power
     * after that it throws a random number in order to check if this packet will be corrupted or not
     *
     * @param frame pointer to the packet
     * @param lteinfo pointer to the user control info
     */
    virtual bool error(LteAirFrame *frame, UserControlInfo* lteI)=0;
    //TODO NOT IMPLEMENTED YET
    virtual bool errorDas(LteAirFrame *frame, UserControlInfo* lteI)=0;
    /*
     * Compute Attenuation caused by pathloss and shadowing (optional)
     *
     * @param nodeid mac node id of UE
     * @param dir traffic direction
     * @param move position of end point comunication (if dir==UL is the position of UE else is the position of eNodeB)
     */
    virtual double getAttenuation(MacNodeId nodeId, Direction dir, inet::Coord coord)=0;
    /*
     * Compute sir for each band for user nodeId according to multipath fading
     *
     * @param frame pointer to the packet
     * @param lteinfo pointer to the user control info
     */
    virtual std::vector<double> getSIR(LteAirFrame *frame, UserControlInfo* lteInfo)=0;
    /*
     * Compute sinr for each band for user nodeId according to pathloss, shadowing (optional) and multipath fading
     *
     * @param frame pointer to the packet
     * @param lteinfo pointer to the user control info
     */
    virtual std::vector<double> getSINR(LteAirFrame *frame, UserControlInfo* lteInfo)=0;
    /*
     * Compute the error probability of the transmitted packet according to cqi used, txmode, and the received power
     * after that it throws a random number in order to check if this packet will be corrupted or not
     *
     * @param frame pointer to the packet
     * @param lteinfo pointer to the user control info
     * @param rsrpVector the received signal for each RB, if it has already been computed
     */
    virtual bool error_D2D(LteAirFrame *frame, UserControlInfo* lteInfo, std::vector<double> rsrpVector)=0;
    /*
     * Compute the error probability of the transmitted packet according to mcs used, txmode, and the received power
     * after that it throws a random number in order to check if this packet will be corrupted or not
     *
     * @param frame pointer to the packet
     * @param lteinfo pointer to the user control info
     * @param rsrpVector the received signal for each RB, if it has already been computed
     * @param mcs the modulation and coding scheme used in sending the message.
     * @returns a tuple specifying whether it was successfully received based on SNR and SINR
     */
    virtual std::tuple<bool, bool> error_Mode4(LteAirFrame *frame, UserControlInfo* lteInfo, std::vector<double> rsrpVector, std::vector<double> sinrVector, int mcs)=0;
    /*
     * Compute Received useful signal for D2D transmissions
     */
    virtual std::tuple<std::vector<double>, double> getRSRP_D2D(LteAirFrame *frame, UserControlInfo* lteInfo_1, MacNodeId destId, inet::Coord destCoord)=0;
    /*
     * Compute sinr (D2D) for each band for user nodeId according to pathloss, shadowing (optional) and multipath fading
     *
     * @param frame pointer to the packet
     * @param lteinfo pointer to the user control info
     */
    virtual std::vector<double> getSINR_D2D(LteAirFrame *frame, UserControlInfo* lteInfo,MacNodeId peerUeId,inet::Coord peerUeCoord,MacNodeId enbId=0)=0;
    virtual std::vector<double> getSINR_D2D(LteAirFrame *frame, UserControlInfo* lteInfo_1, MacNodeId destId, inet::Coord destCoord,MacNodeId enbId,std::vector<double> rsrpVector, bool interference)=0;
    /*
     * Compute RSSI for each band for user nodeId according to pathloss, shadowing (optional) and multipath fading
     *
     * @param frame pointer to the packet
     * @param lteinfo pointer to the user control info
     */
    virtual std::tuple<std::vector<double>, std::vector<double>> getRSSI_SINR(LteAirFrame *frame, UserControlInfo* lteInfo_1, MacNodeId destId, inet::Coord destCoord,MacNodeId enbId,std::vector<double> rsrpVector)=0;

    virtual double getTxRxDistance(UserControlInfo* lteInfo)=0;
};

#endif
