/**************************************************************//**
* Copyright(c) Kunming Dongdian Technology Co. LTD
 *  All rights reserved. 
 *   
 *  @file    TcpServerImpl.h
 *  @brief   This file defines all the callbacks used for TcpServer.
 *   
 *  @version V2.0.0
 *  @author  xucheng
 *  @date    2022/12/22
 *   
 *  @note 更新日志，更新日志每次修改占1行，分别注明修改人、修改时间、
 *  		修改内容
 ******************************************************************/

#pragma once 

#include <memory>
#include <functional>
#include <system_error>

namespace Utils
{
namespace net 
{

class TcpConnectionBase;

using TcpConnPtr = std::shared_ptr<TcpConnectionBase>;

using ConnectionCallback = std::function<void(const TcpConnPtr&)>;
using MessageCallback = std::function<void(const TcpConnPtr&, const uint8_t*, int32_t)>;
using ErrorCallback = std::function<void(const TcpConnPtr&, std::error_code)>;
using CloseCallback = std::function<void(const TcpConnPtr&)>;


} //namespace net 
} //namespace common