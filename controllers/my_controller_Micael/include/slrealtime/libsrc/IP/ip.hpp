/* Copyright 2019-2021 The MathWorks, Inc. */

#ifndef ip_hpp
#define ip_hpp

#if defined(MATLAB_MEX_FILE)

#if defined (_WIN32)
#define IPAPI __declspec(dllexport)
#elif defined (__linux)
#define IPAPI __attribute__((visibility("default")))
#endif

#else
#define IPAPI
#endif

#ifdef __cplusplus

#include    <vector>
#include    <stdio.h>
#include    <string>
#include    <memory>
#include    <algorithm>

namespace slrealtime {
    namespace ip {
       
        #define HostIPAddrForUseHostTargetConn "0.0.0.0"

        class Socket;

        IPAPI Socket *GetSocket(std::string address, uint16_t port);
        IPAPI void ThrowWarning(std::string);
        IPAPI std::string Trim(std::string);

        IPAPI std::string GetTargetIPAddr();

        class SocketList
        {
        private:
            static std::vector<Socket *> sockets;
            SocketList() {}
            ~SocketList() {}
        public:
            static IPAPI Socket *findSocket(std::string localAddress, uint16_t port);
            static IPAPI void addSocket(Socket * socket);
            static IPAPI void removeSocket(Socket * socket);
            static IPAPI int getNumSockets();
        };

        class Socket
        {
        protected:
            std::string localAddress_;
            uint16_t port_ = 0;
            bool disabled_ = false;
           
        public:
            IPAPI Socket(std::string localAddress, uint16_t port);
            IPAPI virtual ~Socket() { SocketList::removeSocket(this); }
            IPAPI std::string localAddress() const { return localAddress_; }
            IPAPI uint16_t port() const { return port_; }
            IPAPI virtual std::string type() = 0;
            IPAPI virtual void disable() = 0;
        };

    }
}

#endif
#endif

