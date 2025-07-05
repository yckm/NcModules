/**************************************************************//**
* Copyright(c) Kunming Dongdian Technology Co. LTD
 *  All rights reserved. 
 *   
 *  @file    TcpServer.h
 *  @brief   This file defins a templated implementation of TcpServer, the
 *           default implementation is AsioAsyncTcpServerImpl, you
 *           can extend it with any other implementaion as long as
 *           you follow the interfaces declared in TcpServerImpl.h
 *   
 *  @version V2.0.0
 *  @author  xucheng
 *  @date    2022/12/22
 *   
 *  @note 更新日志，更新日志每次修改占1行，分别注明修改人、修改时间、
 *  		修改内容
 ******************************************************************/
#pragma once

#include "AsioAsyncTcpServerImpl.h"

namespace Utils
{
namespace net 
{

template<typename ImplType = AsioAsyncTcpServerImpl>
class TcpServer
{
public:
    TcpServer() 
    {
        m_impl = std::make_unique<ImplType>();
    }

    void init(int32_t port, const std::string& listenAddress = "")
    {
        m_impl->init(port, listenAddress);
    }

    bool start()
    {
        return m_impl->start();
    }

    void stop()
    {
        m_impl->stop();
    }

    void setConnectionCallback(ConnectionCallback&& cb)
    {
        m_impl->setConnectionCallback(std::move(cb));
    }

    void setMessageCallback(MessageCallback&& cb)
    {
        m_impl->setMessageCallback(std::move(cb));
    }

    void setErrorCallback(ErrorCallback&& cb)
    {
        m_impl->setErrorCallback(std::move(cb));
    }

private:
    std::unique_ptr<TcpServerImpl> m_impl{nullptr};
};

} //namespace net 
} //namespace common