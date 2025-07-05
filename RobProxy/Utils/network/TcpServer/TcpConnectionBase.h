/**************************************************************//**
* Copyright(c) Kunming Dongdian Technology Co. LTD
 *  All rights reserved. 
 *   
 *  @file    TcpServerImpl.h
 *  @brief   This file defines the base class and interfaces should
 *           be implemented for a TcpConnection.
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
#include "Callbacks.h"

namespace Utils
{
namespace net 
{
class TcpConnectionBase
{
public:
    virtual ~TcpConnectionBase() {}
    
    virtual std::string name() const = 0;
    virtual void start() = 0;
    virtual void close() = 0;
    virtual bool isConnected() const { return m_isConnected; }

    virtual bool send(const uint8_t* data, int32_t length) = 0;

    virtual void setMessageCallback(const MessageCallback& cb)
    {
        m_msgCallback = cb;
    }

    virtual void setCloseCallback(const CloseCallback& cb)
    {
        m_closeCallback = cb;
    }

    virtual void setErrorCallback(const ErrorCallback& cb)
    {
        m_errorCallback = cb;
    }

protected:
    bool m_isConnected{false};
    MessageCallback m_msgCallback{nullptr};
    CloseCallback m_closeCallback{nullptr};
    ErrorCallback m_errorCallback{nullptr};
};


} //namespace net 
} //namespace common