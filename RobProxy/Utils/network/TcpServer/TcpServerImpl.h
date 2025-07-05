/**************************************************************//**
* Copyright(c) Kunming Dongdian Technology Co. LTD
 *  All rights reserved. 
 *   
 *  @file    TcpServerImpl.h
 *  @brief   This file defines the base class and interfaces should
 *           be implemented for a TcpServer.
 *   
 *  @version V2.0.0
 *  @author  xucheng
 *  @date    2022/12/22
 *   
 *  @note 更新日志，更新日志每次修改占1行，分别注明修改人、修改时间、
 *  		修改内容
 ******************************************************************/
#pragma once

#include <cinttypes>
#include <string>


#include "Callbacks.h"
#include "TcpConnectionBase.h"

namespace Utils
{
namespace net 
{

class TcpServerImpl
{
public:
    virtual ~TcpServerImpl() {}

    virtual void init(int32_t port, const std::string& listenAddress = "") = 0;
    virtual bool start() = 0;
    virtual void stop() = 0;

    virtual void setConnectionCallback(ConnectionCallback&& cb) 
    {
        m_connCallback = std::move(cb);
    }

    virtual void setMessageCallback(MessageCallback&& cb)
    {
        m_msgCallback = std::move(cb);
    }

    virtual void setErrorCallback(ErrorCallback&& cb)
    {
        m_errorCallback = std::move(cb);
    }

protected:
    ConnectionCallback m_connCallback{nullptr};
    MessageCallback m_msgCallback{nullptr};
    ErrorCallback m_errorCallback{nullptr};
};

} //namespace net 
} //namespace common