//
//                           SimuLTE
//
// This file is part of a software released under the license included in file
// "license.pdf". This license can be also found at http://www.ltesimulator.com/
// The above file and the present reference are part of the software itself,
// and cannot be removed from it.
//

#include "stack/mac/buffer/harq/LteHarqProcessTx.h"

LteHarqProcessTx::LteHarqProcessTx(unsigned char acid, unsigned int numUnits, unsigned int numProcesses,
    LteMacBase *macOwner, LteMacBase *dstMac)
{
    macOwner_ = macOwner;
    acid_ = acid;
    numHarqUnits_ = numUnits;
    units_ = new UnitVector(numUnits);
    numProcesses_ = numProcesses;
    numEmptyUnits_ = numUnits; //++ @ insert, -- @ unit reset (ack or fourth nack)
    numSelected_ = 0; //++ @ markSelected and insert, -- @ extract/sendDown
    dropped_ = false;

    // H-ARQ unit instances
    for (unsigned int i = 0; i < numHarqUnits_; i++)
    {
        (*units_)[i] = new LteHarqUnitTx(acid, i, macOwner_, dstMac);
    }
}

std::vector<UnitStatus>
LteHarqProcessTx::getProcessStatus()
{
    std::vector<UnitStatus> ret(numHarqUnits_);

    for (unsigned int j = 0; j < numHarqUnits_; j++)
    {
        ret[j].first = j;
        ret[j].second = getUnitStatus(j);
    }
    return ret;
}

void LteHarqProcessTx::insertPdu(LteMacPdu *pdu, Codeword cw)
{
    numEmptyUnits_--;
    numSelected_++;
    (*units_)[cw]->insertPdu(pdu);
    dropped_ = false;
}

void LteHarqProcessTx::markSelected(Codeword cw)
{
    if (numSelected_ == numHarqUnits_)
        throw cRuntimeError("H-ARQ TX process: cannot select another unit because they are all already selected");

    numSelected_++;
    (*units_)[cw]->markSelected();
}

LteMacPdu *LteHarqProcessTx::extractPdu(Codeword cw)
{
    if (numSelected_ == 0)
        throw cRuntimeError("H-ARQ TX process: cannot extract pdu: numSelected = 0 ");

    numSelected_--;
    LteMacPdu *pdu = (*units_)[cw]->extractPdu();
    return pdu;
}

bool LteHarqProcessTx::pduFeedback(HarqAcknowledgment fb, Codeword cw)
{
    // controllare se numempty == numunits e restituire true/false
    bool reset = (*units_)[cw]->pduFeedback(fb);

    if (reset)
    {
        numEmptyUnits_++;
    }

    // return true if the process has become empty
    if (numEmptyUnits_ == numHarqUnits_)
        reset = true;
    else
        reset = false;

    return reset;
}

bool LteHarqProcessTx::selfNack(Codeword cw)
{
    bool reset = (*units_)[cw]->selfNack();

    if (reset)
    {
        numEmptyUnits_++;
    }

    // return true if the process has become empty
    if (numEmptyUnits_ == numHarqUnits_)
        reset = true;
    else
        reset = false;

    return reset;
}

bool LteHarqProcessTx::hasReadyUnits()
{
    for (unsigned int i = 0; i < numHarqUnits_; i++)
    {
        if ((*units_)[i]->isReady())
            return true;
    }
    return false;
}

simtime_t LteHarqProcessTx::getOldestUnitTxTime()
{
    simtime_t oldestTxTime = NOW + 1;
    simtime_t curTxTime = 0;
    for (unsigned int i = 0; i < numHarqUnits_; i++)
    {
        if ((*units_)[i]->isReady())
        {
            curTxTime = (*units_)[i]->getTxTime();
            if (curTxTime < oldestTxTime)
            {
                oldestTxTime = curTxTime;
            }
        }
    }
    return oldestTxTime;
}

CwList LteHarqProcessTx::readyUnitsIds()
{
    CwList ul;

    for (Codeword i = 0; i < numHarqUnits_; i++)
    {
        if ((*units_)[i]->isReady())
        {
            ul.push_back(i);
        }
    }
    return ul;
}

CwList LteHarqProcessTx::emptyUnitsIds()
{
    CwList ul;
    for (Codeword i = 0; i < numHarqUnits_; i++)
    {
        if ((*units_)[i]->isEmpty())
        {
            ul.push_back(i);
        }
    }
    return ul;
}

CwList LteHarqProcessTx::selectedUnitsIds()
{
    CwList ul;
    for (Codeword i = 0; i < numHarqUnits_; i++)
    {
        if ((*units_)[i]->isMarked())
        {
            ul.push_back(i);
        }
    }
    return ul;
}

bool LteHarqProcessTx::isEmpty()
{
    return (numEmptyUnits_ == numHarqUnits_);
}

LteMacPdu *LteHarqProcessTx::getPdu(Codeword cw)
{
    return (*units_)[cw]->getPdu();
}

long LteHarqProcessTx::getPduId(Codeword cw)
{
    return (*units_)[cw]->getMacPduId();
}

void LteHarqProcessTx::forceDropProcess()
{
    for (unsigned int i = 0; i < numHarqUnits_; i++)
    {
        (*units_)[i]->forceDropUnit();
    }
    numEmptyUnits_ = numHarqUnits_;
    numSelected_ = 0;
    dropped_ = true;
}

bool LteHarqProcessTx::forceDropUnit(Codeword cw)
{
    if ((*units_)[cw]->isMarked())
        numSelected_--;

    (*units_)[cw]->forceDropUnit();
    numEmptyUnits_++;

    // empty process?
    return numEmptyUnits_ == numHarqUnits_;
}

TxHarqPduStatus LteHarqProcessTx::getUnitStatus(Codeword cw)
{
    return (*units_)[cw]->getStatus();
}

void LteHarqProcessTx::dropPdu(Codeword cw)
{
    (*units_)[cw]->dropPdu();
    numEmptyUnits_++;
}

bool LteHarqProcessTx::isUnitEmpty(Codeword cw)
{
    return (*units_)[cw]->isEmpty();
}

bool LteHarqProcessTx::isUnitReady(Codeword cw)
{
    return (*units_)[cw]->isReady();
}

unsigned char LteHarqProcessTx::getTransmissions(Codeword cw)
{
    return (*units_)[cw]->getTransmissions();
}

inet::int64 LteHarqProcessTx::getPduLength(Codeword cw)
{
    return (*units_)[cw]->getPduLength();
}

simtime_t LteHarqProcessTx::getTxTime(Codeword cw)
{
    return (*units_)[cw]->getTxTime();
}

bool LteHarqProcessTx::isUnitMarked(Codeword cw)
{
    return (*units_)[cw]->isMarked();
}

bool LteHarqProcessTx::isDropped()
{
    return dropped_;
}

LteHarqProcessTx::~LteHarqProcessTx()
{
    UnitVector::iterator it = units_->begin();
    for (; it != units_->end(); ++it)
         delete *it;

    units_->clear();
    delete units_;
    units_ = NULL;
    macOwner_ = NULL;
}
