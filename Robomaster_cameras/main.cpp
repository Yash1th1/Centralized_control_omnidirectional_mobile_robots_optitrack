//optitrack cameras tracking integrated with omnidirectional robots control
//For networking with the robots winsock is used
#define WIN32_LEAN_AND_MEAN
#define TCP_PROTO
#define _WINSOCK_DEPRECATED_NO_WARNINGS
#include <inttypes.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <windows.h>
#include<fstream>// Header for streaming files used to plot errors
#ifdef _WIN32
#   include <conio.h>
#else
#   include <unistd.h>
#   include <termios.h>
#endif

#include <vector>
#include "NATUtils.h"
#include <NatNetTypes.h>
#include <NatNetCAPI.h>
#include <NatNetClient.h>
#include <math.h>
#include <Eigen/Dense>
#include<cmath>
#include <thread>//chassis headers
#include <chrono>
#include <winsock2.h>
#include <Ws2tcpip.h>
#include "commands.h"
#include<cmath>
#include<mutex> 
#define N 7 //number of Robots
#pragma comment(lib, "Ws2_32.lib")
#ifndef _WIN32
char getch();
#endif
using namespace std;
mutex camera;
mutex Collision;
mutex Cohesion;
void cameraTracking(int argc, char* argv[]);
void _WriteHeader(FILE* fp, sDataDescriptions* pBodyDefs);
void _WriteFrame(FILE* fp, sFrameOfMocapData* data);
void _WriteFooter(FILE* fp);
void NATNET_CALLCONV ServerDiscoveredCallback(const sNatNetDiscoveredServer* pDiscoveredServer, void* pUserContext);
void NATNET_CALLCONV DataHandler(sFrameOfMocapData* data, void* pUserData);    // receives data from the server
void NATNET_CALLCONV MessageHandler(Verbosity msgType, const char* msg);      // receives NatNet error messages
void resetClient();
int ConnectClient();
void robotControl0(const char* addr, const int port);
void robotControl1(const char* addr, const int port);
void robotControl2c(const char* addr, const int port);
void robotControl3c(const char* addr, const int port);
void robotControl4c(const char* addr, const int port);
void robotControl5c(const char* addr, const int port);
void robotControl6c(const char* addr, const int port);
void robotControltest(const char* addr, const int port);
void cohesion_n_collision_control();


static const ConnectionType kDefaultConnectionType = ConnectionType_Multicast;

NatNetClient* g_pClient = NULL;
FILE* g_outputFile;
std::vector< sNatNetDiscoveredServer > g_discoveredServers;
sNatNetClientConnectParams g_connectParams;
char g_discoveredMulticastGroupAddr[kNatNetIpv4AddrStrLenMax] = NATNET_DEFAULT_MULTICAST_ADDRESS;
int g_analogSamplesPerMocapFrame = 0;
sServerDescription g_serverDescription;

// allocate memory in heap for storing the position and orientation of each of the  robot

std::shared_ptr<float> q1x(new float);
std::shared_ptr<float> q1y(new float);
std::shared_ptr<float> q1z(new float);
std::shared_ptr<float> q1w(new float);

std::shared_ptr<float> q2x(new float);
std::shared_ptr<float> q2y(new float);
std::shared_ptr<float> q2z(new float);
std::shared_ptr<float> q2w(new float);

std::shared_ptr<float> q3x(new float);
std::shared_ptr<float> q3y(new float);
std::shared_ptr<float> q3z(new float);
std::shared_ptr<float> q3w(new float);

std::shared_ptr<float> q4x(new float);
std::shared_ptr<float> q4y(new float);
std::shared_ptr<float> q4z(new float);
std::shared_ptr<float> q4w(new float);

std::shared_ptr<float> q5x(new float);
std::shared_ptr<float> q5y(new float);
std::shared_ptr<float> q5z(new float);
std::shared_ptr<float> q5w(new float);

std::shared_ptr<float> q6x(new float);
std::shared_ptr<float> q6y(new float);
std::shared_ptr<float> q6z(new float);
std::shared_ptr<float> q6w(new float);

std::shared_ptr<float> q7x(new float);
std::shared_ptr<float> q7y(new float);
std::shared_ptr<float> q7z(new float);
std::shared_ptr<float> q7w(new float);

std::shared_ptr<float> q8x(new float);
std::shared_ptr<float> q8y(new float);
std::shared_ptr<float> q8z(new float);
std::shared_ptr<float> q8w(new float);

std::shared_ptr<float> q9x(new float);
std::shared_ptr<float> q9y(new float);
std::shared_ptr<float> q9z(new float);
std::shared_ptr<float> q9w(new float);

std::shared_ptr<float> q10x(new float);
std::shared_ptr<float> q10y(new float);
std::shared_ptr<float> q10z(new float);
std::shared_ptr<float> q10w(new float);

std::shared_ptr<double> zpos1(new double);
std::shared_ptr<double> zpos2(new double);
std::shared_ptr<double> zpos3(new double);
std::shared_ptr<double> zpos4(new double);
std::shared_ptr<double> zpos5(new double);
std::shared_ptr<double> zpos6(new double);
std::shared_ptr<double> zpos7(new double);
std::shared_ptr<double> zpos8(new double);
std::shared_ptr<double> zpos9(new double);
std::shared_ptr<double> zpos10(new double);

std::shared_ptr<double> xpos1(new double);
std::shared_ptr<double> xpos2(new double);
std::shared_ptr<double> xpos3(new double);
std::shared_ptr<double> xpos4(new double);
std::shared_ptr<double> xpos5(new double);
std::shared_ptr<double> xpos6(new double);
std::shared_ptr<double> xpos7(new double);
std::shared_ptr<double> xpos8(new double);
std::shared_ptr<double> xpos9(new double);
std::shared_ptr<double> xpos10(new double);

std::shared_ptr<double> ypos1(new double);

std::shared_ptr<Eigen::MatrixXd> U(new Eigen::MatrixXd(N, 2));

std::shared_ptr<Eigen::MatrixXd> F_Ch(new Eigen::MatrixXd(N, 2));

std::shared_ptr<int> flag(new int);

double calculateAngle(double x_A, double y_A, double x_B, double y_B);

void NATNET_CALLCONV ServerDiscoveredCallback(const sNatNetDiscoveredServer* pDiscoveredServer, void* pUserContext)
{
    char serverHotkey = '.';
    if (g_discoveredServers.size() < 9)
    {
        serverHotkey = static_cast<char>('1' + g_discoveredServers.size());
    }

    printf("[%c] %s %d.%d at %s ",
        serverHotkey,
        pDiscoveredServer->serverDescription.szHostApp,
        pDiscoveredServer->serverDescription.HostAppVersion[0],
        pDiscoveredServer->serverDescription.HostAppVersion[1],
        pDiscoveredServer->serverAddress);

    if (pDiscoveredServer->serverDescription.bConnectionInfoValid)
    {
        printf("(%s)\n", pDiscoveredServer->serverDescription.ConnectionMulticast ? "multicast" : "unicast");
    }
    else
    {
        printf("(WARNING: Legacy server, could not autodetect settings. Auto-connect may not work reliably.)\n");
    }

    g_discoveredServers.push_back(*pDiscoveredServer);
}

// Establish a NatNet Client connection
int ConnectClient()
{
    // Release previous server
    g_pClient->Disconnect();

    // Init Client and connect to NatNet server
    int retCode = g_pClient->Connect(g_connectParams);
    if (retCode != ErrorCode_OK)
    {
        printf("Unable to connect to server.  Error code: %d. Exiting.\n", retCode);
        return ErrorCode_Internal;
    }
    else
    {
        // connection succeeded

        void* pResult;
        int nBytes = 0;
        ErrorCode ret = ErrorCode_OK;

        // print server info
        memset(&g_serverDescription, 0, sizeof(g_serverDescription));
        ret = g_pClient->GetServerDescription(&g_serverDescription);
        if (ret != ErrorCode_OK || !g_serverDescription.HostPresent)
        {
            printf("Unable to connect to server. Host not present. Exiting.\n");
            return 1;
        }
        printf("\n[SampleClient] Server application info:\n");
        printf("Application: %s (ver. %d.%d.%d.%d)\n", g_serverDescription.szHostApp, g_serverDescription.HostAppVersion[0],
            g_serverDescription.HostAppVersion[1], g_serverDescription.HostAppVersion[2], g_serverDescription.HostAppVersion[3]);
        printf("NatNet Version: %d.%d.%d.%d\n", g_serverDescription.NatNetVersion[0], g_serverDescription.NatNetVersion[1],
            g_serverDescription.NatNetVersion[2], g_serverDescription.NatNetVersion[3]);
        printf("Client IP:%s\n", g_connectParams.localAddress);
        printf("Server IP:%s\n", g_connectParams.serverAddress);
        printf("Server Name:%s\n", g_serverDescription.szHostComputerName);

        // get mocap frame rate
        ret = g_pClient->SendMessageAndWait("FrameRate", &pResult, &nBytes);
        if (ret == ErrorCode_OK)
        {
            float fRate = *((float*)pResult);
            printf("Mocap Framerate : %3.2f\n", fRate);
        }
        else
            printf("Error getting frame rate.\n");

        // get # of analog samples per mocap frame of data
        ret = g_pClient->SendMessageAndWait("AnalogSamplesPerMocapFrame", &pResult, &nBytes);
        if (ret == ErrorCode_OK)
        {
            g_analogSamplesPerMocapFrame = *((int*)pResult);
            printf("Analog Samples Per Mocap Frame : %d\n", g_analogSamplesPerMocapFrame);
        }
        else
            printf("Error getting Analog frame rate.\n");
    }

    return ErrorCode_OK;
}

// DataHandler receives data from the server
// This function is called by NatNet when a frame of mocap data is available
void NATNET_CALLCONV DataHandler(sFrameOfMocapData* data, void* pUserData)

{

    NatNetClient* pClient = (NatNetClient*)pUserData;

    // Software latency here is defined as the span of time between:
    //   a) The reception of a complete group of 2D frames from the camera system (CameraDataReceivedTimestamp)
    // and
    //   b) The time immediately prior to the NatNet frame being transmitted over the network (TransmitTimestamp)
    //
    // This figure may appear slightly higher than the "software latency" reported in the Motive user interface,
    // because it additionally includes the time spent preparing to stream the data via NatNet.
    const uint64_t softwareLatencyHostTicks = data->TransmitTimestamp - data->CameraDataReceivedTimestamp;
    const double softwareLatencyMillisec = (softwareLatencyHostTicks * 1000) / static_cast<double>(g_serverDescription.HighResClockFrequency);

    // Transit latency is defined as the span of time between Motive transmitting the frame of data, and its reception by the client (now).
    // The SecondsSinceHostTimestamp method relies on NatNetClient's internal clock synchronization with the server using Cristian's algorithm.
    const double transitLatencyMillisec = pClient->SecondsSinceHostTimestamp(data->TransmitTimestamp) * 1000.0;

    if (g_outputFile)
    {
        _WriteFrame(g_outputFile, data);
    }

    int i = 0;

    const bool bSystemLatencyAvailable = data->CameraMidExposureTimestamp != 0;

    // FrameOfMocapData params
    bool bIsRecording = ((data->params & 0x01) != 0);
    bool bTrackedModelsChanged = ((data->params & 0x02) != 0);
    if (bIsRecording)
        printf("RECORDING\n");
    if (bTrackedModelsChanged)
        printf("Models Changed.\n");


    // timecode - for systems with an eSync and SMPTE timecode generator - decode to values
    int hour, minute, second, frame, subframe;
    NatNet_DecodeTimecode(data->Timecode, data->TimecodeSubframe, &hour, &minute, &second, &frame, &subframe);
    // decode to friendly string
    char szTimecode[128] = "";
    NatNet_TimecodeStringify(data->Timecode, data->TimecodeSubframe, szTimecode, 128);
    //printf("Timecode : %s\n", szTimecode);
    // Rigid Bodies
    //printRigidBodies(data);
    //The Rigidbody formed first on the motive is 0, no matter what streaming ID you set for it.
    *q1x = data->RigidBodies[0].qx;
    *q1y = data->RigidBodies[0].qy;
    *q1z = data->RigidBodies[0].qz;
    *q1w = data->RigidBodies[0].qw;

    *q2x = data->RigidBodies[1].qx;
    *q2y = data->RigidBodies[1].qy;
    *q2z = data->RigidBodies[1].qz;
    *q2w = data->RigidBodies[1].qw;

    *q3x = data->RigidBodies[2].qx;
    *q3y = data->RigidBodies[2].qy;
    *q3z = data->RigidBodies[2].qz;
    *q3w = data->RigidBodies[2].qw;

    *q4x = data->RigidBodies[3].qx;
    *q4y = data->RigidBodies[3].qy;
    *q4z = data->RigidBodies[3].qz;
    *q4w = data->RigidBodies[3].qw;

    *q5x = data->RigidBodies[4].qx;
    *q5y = data->RigidBodies[4].qy;
    *q5z = data->RigidBodies[4].qz;
    *q5w = data->RigidBodies[4].qw;

    *q6x = data->RigidBodies[5].qx;
    *q6y = data->RigidBodies[5].qy;
    *q6z = data->RigidBodies[5].qz;
    *q6w = data->RigidBodies[5].qw;

    *q7x = data->RigidBodies[6].qx;
    *q7y = data->RigidBodies[6].qy;
    *q7z = data->RigidBodies[6].qz;
    *q7w = data->RigidBodies[6].qw;

    *q8x = data->RigidBodies[7].qx;
    *q8y = data->RigidBodies[7].qy;
    *q8z = data->RigidBodies[7].qz;
    *q8w = data->RigidBodies[7].qw;

    *q9x = data->RigidBodies[8].qx;
    *q9y = data->RigidBodies[8].qy;
    *q9z = data->RigidBodies[8].qz;
    *q9w = data->RigidBodies[8].qw;

    *q10x = data->RigidBodies[9].qx;
    *q10y = data->RigidBodies[9].qy;
    *q10z = data->RigidBodies[9].qz;
    *q10w = data->RigidBodies[9].qw;

    *zpos1 = data->RigidBodies[0].z;
    *zpos2 = data->RigidBodies[1].z;
    *zpos3 = data->RigidBodies[2].z;
    *zpos4 = data->RigidBodies[3].z;
    *zpos5 = data->RigidBodies[4].z;
    *zpos6 = data->RigidBodies[5].z;
    *zpos7 = data->RigidBodies[6].z;
    *zpos8 = data->RigidBodies[7].z;
    *zpos9 = data->RigidBodies[8].z;
    *zpos10 = data->RigidBodies[9].z;

    *ypos1 = data->RigidBodies[0].y;

    *xpos1 = data->RigidBodies[0].x;
    *xpos2 = data->RigidBodies[1].x;
    *xpos3 = data->RigidBodies[2].x;
    *xpos4 = data->RigidBodies[3].x;
    *xpos5 = data->RigidBodies[4].x;
    *xpos6 = data->RigidBodies[5].x;
    *xpos7 = data->RigidBodies[6].x;
    *xpos8 = data->RigidBodies[7].x;
    *xpos9 = data->RigidBodies[8].x;
    *xpos10 = data->RigidBodies[9].x;

}


// MessageHandler receives NatNet error/debug messages
void NATNET_CALLCONV MessageHandler(Verbosity msgType, const char* msg)
{
    // Optional: Filter out debug messages
    if (msgType < Verbosity_Info)
    {
        return;
    }

    printf("\n[NatNetLib]");

    switch (msgType)
    {
    case Verbosity_Debug:
        printf(" [DEBUG]");
        break;
    case Verbosity_Info:
        printf("  [INFO]");
        break;
    case Verbosity_Warning:
        printf("  [WARN]");
        break;
    case Verbosity_Error:
        printf(" [ERROR]");
        break;
    default:
        printf(" [?????]");
        break;
    }

    printf(": %s\n", msg);
}


/* File writing routines */
void _WriteHeader(FILE* fp, sDataDescriptions* pBodyDefs)
{
    int i = 0;

    if (pBodyDefs->arrDataDescriptions[0].type != Descriptor_MarkerSet)
        return;

    sMarkerSetDescription* pMS = pBodyDefs->arrDataDescriptions[0].Data.MarkerSetDescription;

    fprintf(fp, "<MarkerSet>\n\n");
    fprintf(fp, "<Name>\n%s\n</Name>\n\n", pMS->szName);

    fprintf(fp, "<Markers>\n");
    for (i = 0; i < pMS->nMarkers; i++)
    {
        fprintf(fp, "%s\n", pMS->szMarkerNames[i]);
    }
    fprintf(fp, "</Markers>\n\n");

    fprintf(fp, "<Data>\n");
    fprintf(fp, "Frame#\t");
    for (i = 0; i < pMS->nMarkers; i++)
    {
        fprintf(fp, "M%dX\tM%dY\tM%dZ\t", i, i, i);
    }
    fprintf(fp, "\n");

}


void _WriteFrame(FILE* fp, sFrameOfMocapData* data)
{
    fprintf(fp, "%d", data->iFrame);
    for (int i = 0; i < data->MocapData->nMarkers; i++)
    {
        fprintf(fp, "\t%.5f\t%.5f\t%.5f", data->MocapData->Markers[i][0], data->MocapData->Markers[i][1], data->MocapData->Markers[i][2]);
    }
    fprintf(fp, "\n");
}


void _WriteFooter(FILE* fp)
{
    fprintf(fp, "</Data>\n\n");
    fprintf(fp, "</MarkerSet>\n");
}

void resetClient()
{
    int iSuccess;

    printf("\n\nre-setting Client\n\n.");

    iSuccess = g_pClient->Disconnect();
    if (iSuccess != 0)
        printf("error un-initting Client\n");

    iSuccess = g_pClient->Connect(g_connectParams);
    if (iSuccess != 0)
        printf("error re-initting Client\n");
}

SOCKET createSocket(const char* addr, const int port) {
    int iResult;
    WSADATA wsaData;
    SOCKET ConnectSocket = INVALID_SOCKET;
    struct sockaddr_in clientService;
    std::cout << "Sending command to " << addr << std::endl;

    iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
    if (iResult != NO_ERROR) {
        wprintf(L"WSAStartup failed with error: %d\n", iResult);
        return INVALID_SOCKET;
    }

    ConnectSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (ConnectSocket == INVALID_SOCKET) {
        wprintf(L"socket failed with error: %ld\n", WSAGetLastError());
        WSACleanup();
        return INVALID_SOCKET;
    }

    // Convert IP address string to binary format
    struct sockaddr_in serverAddress;
    if (inet_pton(AF_INET, addr, &(serverAddress.sin_addr)) <= 0) {
        perror("Invalid address");
        closesocket(ConnectSocket); // Close the socket before returning
        WSACleanup();
        return INVALID_SOCKET;
    }

    clientService.sin_family = AF_INET;
    clientService.sin_addr = serverAddress.sin_addr; // Use the converted address
    clientService.sin_port = htons(port);

    iResult = connect(ConnectSocket, (SOCKADDR*)&clientService, sizeof(clientService));
    if (iResult == SOCKET_ERROR) {
        wprintf(L"connect failed with error: %d\n", WSAGetLastError());
        closesocket(ConnectSocket);
        WSACleanup();
        return INVALID_SOCKET;
    }

    return ConnectSocket;
}

void sendCommand(SOCKET ConnectSocket, const std::string& command) {
    const int recvbuflen = 1024;
    char recvbuf[recvbuflen] = "";

    int iResult = send(ConnectSocket, command.c_str(), (int)command.size(), 0);
    if (iResult == SOCKET_ERROR) {
        wprintf(L"send failed with error: %d\n", WSAGetLastError());
        return;
    }

    iResult = recv(ConnectSocket, recvbuf, recvbuflen, MSG_PUSH_IMMEDIATE);
    std::cout << recvbuf << std::endl;
}

void sendWheelCommand(int ConnectSocket, int w1, int w2, int w3, int w4) {
    char command[100]; // Adjustment for the buffer size as needed

    // Using the snprintf to format the command string with dynamic speed values
    snprintf(command, sizeof(command), "wheels(%d, %d, %d, %d)", w1, w2, w3, w4);

    // Sending the dynamically generated command
    sendCommand(ConnectSocket, command);
}

double calculateAngle(double x_A, double y_A, double x_B, double y_B) {
    double dist = sqrt(pow((y_A - y_B), 2) + pow((x_A - x_B), 2));
    // Calculate the angle using atan2
    return std::atan2(((x_A - x_B)), ((y_A - y_B)));
}
std::pair<std::vector<double>, std::vector<double>> generateSineWavePoints(double x0, double z0, double x1, double z1, double amplitude, double frequency, int numPoints) {
    std::vector<double> xPoints;
    std::vector<double> zPoints;

    for (int i = 0; i < numPoints; ++i) {
        double t = static_cast<double>(i) / static_cast<double>(numPoints - 1);
        double z = z0 + (z1 - z0) * t;
        double  x = x0 + (x1 - x0) * t + amplitude * std::sin(2 * 3.14159 * frequency * t);

        xPoints.push_back(x);
        zPoints.push_back(z);
    }

    return std::make_pair(xPoints, zPoints);
}
double square_distance(double z1, double x1, double z2, double x2) {
    return (z1 - z2) * (z1 - z2) + (x1 - x2) * (x1 - x2);
}
double distance(double z1, double x1, double z2, double x2) {
    return sqrt((z1 - z2) * (z1 - z2) + (x1 - x2) * (x1 - x2));
}

//Modified Reynolds Flocking Algorithm
void cohesion_n_collision_control() {
    bool live = true;
    double a, r, r_l, r_a, r_l_c, r_a_c, L, eps, vz_c_max, vx_c_max, vz_max, vx_max;
    int count;
    int l = 0;              // index of the leader
    r_l = 0.500;        // Sensing radius of the leader for separation.
    r_a = 0.380;        //sensing radius of the agent for separation.
    r_l_c = 2.00;        // Sensing radius of the leader for cohession.
    r_a_c = 0.550;  // Sensing radius of the agent for cohession.
    L = 15.00;       // weight of the leader
    eps = 0.1;  // small value to avoid division over 0
    vz_max = 25; //max speed in m/s in z direction
    vx_max = 25;//max speed in m/s in x direction
    vz_c_max = 45; //max speed in m/s in z direction
    vx_c_max = 45;
    Eigen::MatrixXd d(N, 2);
    Eigen::MatrixXd Q(N, N);
    Eigen::MatrixXd D(N, N);
    Eigen::MatrixXd J(N, N);
    Eigen::MatrixXd F(N, N);
    Eigen::MatrixXd P(N, 2);
    Eigen::MatrixXd H(N, 2);

    Eigen::MatrixXd c(N, N);
    Eigen::VectorXd c_l(N);

    while (live) {
        unique_lock<mutex> Collisionlock(Collision);
        P.setZero();
        H.setZero();
        D.setZero();
        (*F_Ch).setZero(); //initialize cohesive force to 0
        (*U).setZero();    //initialize separation force to 0
        c.setZero();
        c_l.setZero();
        d << *zpos1, * xpos1,
            * zpos2, * xpos2,
            * zpos3, * xpos3,
            * zpos4, * xpos4,
            * zpos5, * xpos5,
            * zpos6, * xpos6,
            * zpos7, * xpos7;
        // Assuming the robot 1 is the leader
        for (int i = 0; i < N; i++) {
            for (int j = 0; j < N; j++) {
                d << *zpos1, * xpos1,
                    * zpos2, * xpos2,
                    * zpos3, * xpos3,
                    * zpos4, * xpos4,
                    * zpos5, * xpos5,
                    * zpos6, * xpos6,
                    * zpos7, * xpos7;
                D(i, j) = square_distance(d(i, 0), d(i, 1), d(j, 0), d(j, 1));                // Assign distance to matrix
                if (i != j && D(i, j) <= r_a * r_a) {
                    c(i, j) = 1;
                }
                else {
                    c(i, j) = 0;
                }
                if (i != l && D(i, l) <= r_l_c * r_l_c) {
                    c_l(i) = 1;
                }
                else {
                    c_l(i) = 0;
                }
            }

        }
        Eigen::VectorXd rowSum = c.rowwise().sum();
        for (int i = 0; i < N; i++) {
            if (rowSum(i) || c_l(i)) {
                for (int j = 0; j < N; j++) {
                    d << *zpos1, * xpos1,
                        * zpos2, * xpos2;
                    if (i != l && i != j && c_l(i)) {
                        P(i, 0) += ((d(j, 0) + (L * c_l(i) * d(l, 0))) / (rowSum(i) + (L * c_l(i))));
                        P(i, 1) += ((d(j, 1) + (L * c_l(i) * d(l, 1))) / (rowSum(i) + (L * c_l(i))));

                    }


                    if (i != l && j != i && rowSum(i) && c(i, j))
                    {
                        H(i, 0) += 1 / (d(i, 0) - d(j, 0));
                        H(i, 1) += 1 / (d(i, 1) - d(j, 1));
                        (*U)(i, 0) = (H(i, 0) / abs(H(i, 0))) * vz_c_max;
                        (*U)(i, 1) = (H(i, 1) / abs(H(i, 1))) * vx_c_max;
                    }
                }
                P(i, 0) -= d(i, 0);
                P(i, 1) -= d(i, 1);
                (*F_Ch)(i, 0) = (P(i, 0) / abs(P(i, 0))) * vz_max;
                (*F_Ch)(i, 1) = (P(i, 1) / abs(P(i, 1))) * vx_max;
            }
            else {
                (*F_Ch)(i, 0) = 0;
                (*F_Ch)(i, 1) = 0;
                (*U)(i, 0) = 0;
                (*U)(i, 1) = 0;
            }

        }
        //std::cout << "zpos : " << *zpos2 << std::endl;
        //std::cout << "\n xpos: " << *xpos2 << std::endl;
        Sleep(20);
        Collisionlock.unlock();
    }
}

//Simple motion robot control with trajectory with heading towards the goal

void robotControl0(const char* addr, const int port) {
    std::cout << "Z:" << *zpos1 << "\n Xpos: " << *xpos1 << std::endl;
    SOCKET ConnectSocket = createSocket(addr, port);
    if (ConnectSocket == INVALID_SOCKET) {
        return;
    }
    int order = EulOrdYXZr;
    const double pi = 3.1415925635897;
    Quat q(*q1x, *q1y, *q1z, *q1w);
    EulerAngles ea = Eul_FromQuat(q, order);
    bool live = true;
    double eps = 0.09;
    double eps1 = 0.09;
    double kp;
    double ki;
    double kd;
    double xref = -0.600;
    double zref = 2.000;
    double yawref = 0 * (pi / 180);
    double yawref1 = calculateAngle(*xpos1, *zpos1, xref, zref);
    double dt = 0.0045;
    double yaweps = (5 * pi) / 180;
    double yaweps1 = (10 * pi) / 180;
    double yaw1, yawerror1, yawintegral1 = 0, prev_yaw_error1 = 0, yawspeed1, W_yaw1;
    double kpx = 1;
    double kix = 0.5;
    double kdx = 0.25;
    double error, integral = 0, preverror = 0, speed, W;
    // Set sine wave parameters
    double amplitude = 1.000;
    double frequency = 1.0;
    // Set the number of points you want on the sine wave
    int numPoints = 60;
    int i;
    // Generate the sequence of points on the sine wave between A and B
    std::pair<std::vector<double>, std::vector<double>> result = generateSineWavePoints(*xpos1, *zpos1, xref, zref, amplitude, frequency, numPoints);
    std::vector<double> xref1 = result.first;
    std::vector<double> zref1 = result.second;
    //std::cout << "Current angle: " << RadiansToDegrees(ea.x) << std::endl;
    kp = 35;
    ki = 5.55;
    kd = 3.55;
    std::cout << "angle need to be reached " << RadiansToDegrees(yawref1) << std::endl;
    //heading correction to allign in the direction of the target to avoid the escape from camera.
    //Verified no problem with heading correction
    while (live) {
        yawref1 = calculateAngle(*xpos1, *zpos1, xref, zref);
        Quat q(*q1x, *q1y, *q1z, *q1w);
        EulerAngles ea = Eul_FromQuat(q, order);
        yaw1 = ea.x;
        std::cout << RadiansToDegrees(yaw1) << std::endl;
        //Sleep(400);
        yawerror1 = (yawref1 - yaw1);//making it impending towards the target
        //std::cout << "yaw Error: " << yawerror1 << std::endl;
        yawintegral1 += yawerror1;
        yawspeed1 = kp * yawerror1 + (ki * yawintegral1 * dt) + (kd * (yawerror1 - prev_yaw_error1) / dt);
        W_yaw1 = (1 / 0.0325) * (0.126 * yawspeed1);//wheel radius 32.5 mm and lx + ly = 126 mm.
        //std::cout << "wheel speed in rpm: " << W << std::endl;
        if (abs(yawerror1) <= yaweps) {
            std::cout << "here_stop" << std::endl;
            ConnectSocket = createSocket(addr, port);
            sendWheelCommand(ConnectSocket, 0, 0, 0, 0);
            //closing the socket 
            closesocket(ConnectSocket);
            WSACleanup();
            break;
            //std::this_thread::sleep_for(std::chrono::milliseconds(400));
        }
        else {
            ConnectSocket = createSocket(addr, port);
            sendWheelCommand(ConnectSocket, -yawspeed1 / 2, yawspeed1 / 2, -yawspeed1 / 2, yawspeed1 / 2);
            //closing the socket
            closesocket(ConnectSocket);
            WSACleanup();
            //std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        prev_yaw_error1 = yawerror1;

    }

    for (int i = 0; i < numPoints; i++) {
        double dist = sqrt(pow((*zpos1 - zref1[i]), 2) + pow((*xpos1 - xref1[i]), 2));
        while (live) {
            Quat q(*q1x, *q1y, *q1z, *q1w);
            EulerAngles ea = Eul_FromQuat(q, order);
            yawref1 = calculateAngle(*xpos1, *zpos1, xref1[i], zref1[i]);
            double yaw1, yawerror1, yawintegral1 = 0, prev_yaw_error1 = 0, yawspeed1, W_yaw1;
            if (abs(ea.x - yawref1) >= yaweps1) {
                std::cout << "here_stop" << std::endl;
                while (live) {
                    yawref1 = calculateAngle(*xpos1, *zpos1, xref1[i], zref1[i]);
                    Quat q(*q1x, *q1y, *q1z, *q1w);
                    EulerAngles ea = Eul_FromQuat(q, order);
                    yaw1 = ea.x;
                    std::cout << RadiansToDegrees(yaw1) << std::endl;
                    //Sleep(400);
                    yawerror1 = (yawref1 - yaw1);//making it impending towards the target
                    //std::cout << "yaw Error: " << yawerror1 << std::endl;
                    yawintegral1 += yawerror1;
                    yawspeed1 = kp * yawerror1 + (ki * yawintegral1 * dt) + (kd * (yawerror1 - prev_yaw_error1) / dt);
                    W_yaw1 = (1 / 0.0325) * (0.126 * yawspeed1);//wheel radius 32.5 mm and lx + ly = 126 mm.
                    //std::cout << "wheel speed in rpm: " << W << std::endl;
                    if (abs(yawerror1) <= yaweps) {
                        break;
                        //std::this_thread::sleep_for(std::chrono::milliseconds(400));
                    }
                    else {
                        ConnectSocket = createSocket(addr, port);
                        sendWheelCommand(ConnectSocket, -yawspeed1 / 2, yawspeed1 / 2, -yawspeed1 / 2, yawspeed1 / 2);
                        //closing the socket
                        closesocket(ConnectSocket);
                        WSACleanup();
                        //std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    }

                    prev_yaw_error1 = yawerror1;

                }
            }
            else {
                bool flag = (abs(*zpos1 - zref1[i]) <= eps1) && (abs(*xpos1 && xref1[i]) <= eps1);
                std::cout << "Current angle: " << RadiansToDegrees(ea.x) << std::endl;
                std::cout << "\nangle need to be reached " << RadiansToDegrees(yawref1) << std::endl;
                dist = sqrt(pow((*zpos1 - zref1[i]), 2) + pow((*xpos1 - xref1[i]), 2));
                std::cout << "\ndistance: " << dist << std::endl;
                error = dist;
                integral += error;
                speed = (kpx * error) + (kix * integral * dt) + (kdx * (error - preverror) / dt);
                W = (1 / 0.030) * (0.126 * speed);//wheel radius 32.5 mm and lx + ly = 126 mm.
                if (abs(dist) <= eps || flag) {
                    std::cout << "here_stop" << std::endl;
                    ConnectSocket = createSocket(addr, port);
                    sendWheelCommand(ConnectSocket, 0, 0, 0, 0);
                    //closing the socket 
                    closesocket(ConnectSocket);
                    WSACleanup();
                    break;

                }
                else {

                    ConnectSocket = createSocket(addr, port);
                    sendWheelCommand(ConnectSocket, W, W, W, W);
                    //closing the socket 
                    closesocket(ConnectSocket);
                    WSACleanup();

                }

            }
        }
    }
    double yaw, yawerror, yawintegral = 0, prev_yaw_error = 0, yawspeed, W_yaw;
    kp = 35;
    ki = 0.05;
    kd = 3.55;
    while (live) {
        //std::cout << "Z pos: " << *zpos0 << std::endl;
        //std::cout << "x pos: " << *xpos0 << std::endl; 
        //std::cout << "orientation about y (Yaw): " << (*rx0).y << std::endl;
        Quat q(*q1x, *q1y, *q1z, *q1w);
        EulerAngles ea = Eul_FromQuat(q, order);
        yaw = ea.x;
        //std::cout << RadiansToDegrees(yaw) << std::endl;
        //Sleep(400);
        yawerror = (yawref - yaw);
        //std::cout << "yaw Error: " << yawerror << std::endl;
        //error_yawFile << yawerror << std::endl;
        yawintegral += yawerror;
        yawspeed = kp * yawerror + (ki * yawintegral * dt) + (kd * (yawerror - prev_yaw_error) / dt);
        W_yaw = (1 / 0.030) * (0.126 * yawspeed);//wheel radius 32.5 mm and lx + ly = 126 mm.
        //std::cout << "wheel speed in rpm: " << W << std::endl;
        if (abs(yawerror) <= yaweps1) {
            std::cout << "here_stop" << std::endl;
            ConnectSocket = createSocket(addr, port);
            sendWheelCommand(ConnectSocket, 0, 0, 0, 0);
            closesocket(ConnectSocket);
            WSACleanup();
            break;
            //std::this_thread::sleep_for(std::chrono::milliseconds(400));
        }
        else {
            ConnectSocket = createSocket(addr, port);
            sendWheelCommand(ConnectSocket, -yawspeed / 2, yawspeed / 2, -yawspeed / 2, yawspeed / 2);
            closesocket(ConnectSocket);
            WSACleanup();
            //std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        prev_yaw_error = yawerror;

    }
}

void robotControltest(const char* addr, const int port) {
    *flag = 0;
    SOCKET ConnectSocket;
    int order = EulOrdYXZr;
    const double pi = 3.1415925635897;
    Quat q(*q1x, *q1y, *q1z, *q1w);
    EulerAngles ea = Eul_FromQuat(q, order);
    bool live = true;
    double kp;
    double ki;
    double kd;
    double xref = 0.000;
    double zref = -2.000;
    double yawref = (0 * pi) / 180;
    double yawref1 = 0;
    double dt = 0.0045;
    Eigen::MatrixXd ref(3, 1);
    double eps = 0.1;
    double z_bound = 2.3300;
    double x_bound = 2.000;
    double yaweps1 = (10 * pi) / 180;
    double yaweps = (10 * pi) / 180;
    Eigen::MatrixXd W(4, 1);
    Eigen::MatrixXd pos(3, 1);
    Eigen::MatrixXd mapmat(4, 3);
    mapmat << 1, -1, -0.126,
        1, 1, 0.126,
        1, 1, -0.126,
        1, -1, 0.126;
    Eigen::MatrixXd error(3, 1);
    Eigen::MatrixXd preverror(3, 1);
    preverror << 0, 0, 0;
    Eigen::MatrixXd integral(3, 1);
    integral << 0, 0, 0;
    Eigen::MatrixXd control(3, 1);
    Eigen::MatrixXd Collision_control(3, 1);
    Eigen::MatrixXd perror(3, 1);
    Eigen::MatrixXd ierror(3, 1);
    Eigen::MatrixXd derror(3, 1);
    // Set sine wave parameters
    double amplitude = 0.500;
    double frequency = 1.0;
    // Set the number of points you want on the sine wave
    int numPoints = 60;
    int i;
    // Generate the sequence of points on the sine wave between A and B
    std::pair<std::vector<double>, std::vector<double>> result = generateSineWavePoints(*xpos1, *zpos1, xref, zref, amplitude, frequency, numPoints);
    std::vector<double> xref1 = result.first;
    std::vector<double> zref1 = result.second;
    std::ofstream ActualFile("t_Sinewave_trajectory_actual_1.txt");
    std::ofstream GeneratedFile("t_Sinewave_trajectory_generated_1 .txt");

    double yaw1, yawerror1, yawintegral1 = 0, prev_yaw_error1 = 0, yawspeed1, W_yaw1;
    kp = 20;
    ki = 0;
    kd = 3.55;

    //heading correction to allign in the direction of the target to avoid the escape from camera.
    //Verified no problem with heading correction
    while (live) {
        Quat q(*q1x, *q1y, *q1z, *q1w);
        EulerAngles ea = Eul_FromQuat(q, order);
        yaw1 = ea.x;
        yawerror1 = (yawref1 - yaw1);//making it impending towards the target
        yawintegral1 += yawerror1;
        yawspeed1 = kp * yawerror1 + (ki * yawintegral1 * dt) + (kd * (yawerror1 - prev_yaw_error1) / dt);
        W_yaw1 = (1 / 0.0325) * (0.126 * yawspeed1);//wheel radius 32.5 mm and lx + ly = 126 mm.
        //std::cout << "wheel speed in rpm: " << W << std::endl;
        if (abs(yawerror1) <= yaweps) {
            std::cout << "here_stop" << std::endl;
            ConnectSocket = createSocket(addr, port);
            sendWheelCommand(ConnectSocket, 0, 0, 0, 0);
            //closing the socket 
            closesocket(ConnectSocket);
            WSACleanup();
            break;
            //std::this_thread::sleep_for(std::chrono::milliseconds(400));
        }
        else {
            ConnectSocket = createSocket(addr, port);
            sendWheelCommand(ConnectSocket, -yawspeed1 / 2, yawspeed1 / 2, -yawspeed1 / 2, yawspeed1 / 2);
            //closing the socket
            closesocket(ConnectSocket);
            WSACleanup();
            //std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        prev_yaw_error1 = yawerror1;

    }
    //Sleep(1000);

    double kpx = 1.5;
    double kix = 0.05;
    double kdx = 0;

    double kpz = 1.5;
    double kiz = 0.05;
    double kdz = 0;

    double kpo = 1.5;
    double kio = 0.05;
    double kdo = 0;
    Eigen::MatrixXd kp1(3, 3);
    Eigen::MatrixXd ki1(3, 3);
    Eigen::MatrixXd kd1(3, 3);
    kp1 << kpz, 0, 0,
        0, kpx, 0,
        0, 0, kpo;
    ki1 << kiz, 0, 0,
        0, kix, 0,
        0, 0, kio;
    kd1 << kdz, 0, 0,
        0, kdx, 0,
        0, 0, kdo;
    ref << zref, xref, yawref1;


    ref << zref, xref, yawref1;

    if (abs(*zpos1) - z_bound >= 0.01 || abs(*xpos1) - x_bound >= 0.01)
    {
        std::cout << "\nHorcy2 is going out of arena stopping!" << std::endl;
        ConnectSocket = createSocket(addr, port);
        sendWheelCommand(ConnectSocket, 0, 0, 0, 0);
        closesocket(ConnectSocket);
        WSACleanup();


    }
    else {
        while (live) {

            Quat q(*q1x, *q1y, *q1z, *q1w);
            ea = Eul_FromQuat(q, order);
            if (abs(ea.x - (yawref1)) >= (30 * pi / 180)) {
                while (live) {
                    Quat q(*q1x, *q1y, *q1z, *q1w);
                    ea = Eul_FromQuat(q, order);
                    yaw1 = ea.x;
                    //std::cout << RadiansToDegrees(yaw1) << std::endl;
                    //Sleep(400);
                    yawerror1 = (yawref1 - yaw1);//making it impending towards the target
                    //std::cout << "yaw Error: " << yawerror1 << std::endl;
                    yawintegral1 += yawerror1;
                    yawspeed1 = kp * yawerror1 + (ki * yawintegral1 * dt) + (kd * (yawerror1 - prev_yaw_error1) / dt);
                    W_yaw1 = (1 / 0.0325) * (0.126 * yawspeed1);//wheel radius 32.5 mm and lx + ly = 126 mm.
                    //std::cout << "wheel speed in rpm: " << W << std::endl;
                    if (abs(yawerror1) <= yaweps1) {

                        break;

                    }
                    else {
                        ConnectSocket = createSocket(addr, port);
                        sendWheelCommand(ConnectSocket, -yawspeed1 / 2, yawspeed1 / 2, -yawspeed1 / 2, yawspeed1 / 2);
                        //closing the socket
                        closesocket(ConnectSocket);
                        WSACleanup();
                        //std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    }

                    prev_yaw_error1 = yawerror1;

                }
            }
            else {
                ActualFile << *xpos1 << " " << *zpos1 << "\n";
                pos << *zpos1, * xpos1, ea.x;
                error = (ref - pos);//
                //std::cout << "Error: " << error << std::endl;

                integral += error;

                perror = (kp1 * error);
                ierror = ki1 * integral * dt;
                derror = (kd1 * (error - preverror) / dt);
                control = perror + ierror + derror;
                /*Collision_control << (*U)(0, 0), (*U)(0, 1), -ea.x;
                //std::cout<<"Control matrix:\n "<< control<< std::endl;
               if ((*U)(0, 0) != 0 || (*U)(0, 1) != 0) {
                    W = (1 / 0.030) * (mapmat * Collision_control);
                }*/
                W = (1 / 0.030) * (mapmat * control);             //wheel radius 32.5 mm and lx + ly = 126 mm
                //wheel radius 32.5 mm and lx + ly = 126 mm.
                //std::cout << "wheel speed in rpm: " << W << std::endl;

                if (sqrt(error(0, 0) * error(0, 0) + error(1, 0) * error(1, 0)) <= eps)
                {
                    std::cout << "Position error below eps" << std::endl;
                    //ConnectSocket = createSocket(addr, port);
                    //sendWheelCommand(ConnectSocket, 0, 0, 0, 0);
                    //closesocket(ConnectSocket);
                    //WSACleanup();
                    break;

                }
                else {
                    ConnectSocket = createSocket(addr, port);
                    sendWheelCommand(ConnectSocket, W(0, 0), W(1, 0), W(2, 0), W(3, 0));
                    closesocket(ConnectSocket);
                    WSACleanup();

                }

                preverror = error;

            }

        }
    }

    ActualFile.close();
    GeneratedFile.close();
    // errorxFile.close();
    //errorzFile.close();
    //PID to reach the final orientation
    double yaw, yawerror, yawintegral = 0, prev_yaw_error = 0, yawspeed, W_yaw;
    kp = 20;
    ki = 0;
    kd = 3.55;
    while (live) {
        if (abs(abs(*zpos1) - z_bound) <= eps || abs(abs(*xpos1) - x_bound) <= eps)
        {
            std::cout << "\nleader is going out of arena stopping!" << std::endl;
            std::cout << "zpos:" << *zpos1 << " xpos:" << *xpos1 << std::endl;
            ConnectSocket = createSocket(addr, port);
            sendWheelCommand(ConnectSocket, 0, 0, 0, 0);
            closesocket(ConnectSocket);
            WSACleanup();
            break;

        }
        else {
            Quat q(*q1x, *q1y, *q1z, *q1w);
            EulerAngles ea = Eul_FromQuat(q, order);
            yaw = ea.x;
            //std::cout << RadiansToDegrees(yaw) << std::endl;
            //Sleep(400);
            yawerror = (yawref - yaw);
            //std::cout << "yaw Error: " << yawerror << std::endl;
            //error_yawFile << yawerror << std::endl;
            yawintegral += yawerror;
            yawspeed = kp * yawerror + (ki * yawintegral * dt) + (kd * (yawerror - prev_yaw_error) / dt);
            W_yaw = (1 / 0.0325) * (0.126 * yawspeed);//wheel radius 32.5 mm and lx + ly = 126 mm.
            //std::cout << "wheel speed in rpm: " << W << std::endl;
            if (abs(yawerror) <= yaweps) {
                std::cout << "here_yaw_stop" << std::endl;
                ConnectSocket = createSocket(addr, port);
                sendWheelCommand(ConnectSocket, 0, 0, 0, 0);
                closesocket(ConnectSocket);
                WSACleanup();
                Sleep(1000);
                *flag = 1;
                break;
                //std::this_thread::sleep_for(std::chrono::milliseconds(400));
            }
            else {
                ConnectSocket = createSocket(addr, port);
                sendWheelCommand(ConnectSocket, -yawspeed / 2, yawspeed / 2, -yawspeed / 2, yawspeed / 2);
                closesocket(ConnectSocket);
                WSACleanup();
                //std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }

            prev_yaw_error = yawerror;
        }

    }
    //error_yawFile.close();

}

//Trajectory Sine wave in omnidirectional
void robotControl1(const char* addr, const int port) {
    *flag = 0;
    SOCKET ConnectSocket;
    int order = EulOrdYXZr;
    const double pi = 3.1415925635897;
    Quat q(*q1x, *q1y, *q1z, *q1w);
    EulerAngles ea = Eul_FromQuat(q, order);
    bool live = true;
    double kp;
    double ki;
    double kd;
    double xref = 0.000;
    double zref = -2.000;
    double yawref = (0 * pi) / 180;
    double yawref1 = 0;
    double dt = 0.0045;
    Eigen::MatrixXd ref(3, 1);
    double eps = 0.1;
    double z_bound = 2.3300;
    double x_bound = 2.000;
    double yaweps1 = (10 * pi) / 180;
    double yaweps = (10 * pi) / 180;
    Eigen::MatrixXd W(4, 1);
    Eigen::MatrixXd pos(3, 1);
    Eigen::MatrixXd mapmat(4, 3);
    mapmat << 1, -1, -0.126,
        1, 1, 0.126,
        1, 1, -0.126,
        1, -1, 0.126;
    Eigen::MatrixXd error(3, 1);
    Eigen::MatrixXd preverror(3, 1);
    preverror << 0, 0, 0;
    Eigen::MatrixXd integral(3, 1);
    integral << 0, 0, 0;
    Eigen::MatrixXd control(3, 1);
    Eigen::MatrixXd Collision_control(3, 1);
    Eigen::MatrixXd perror(3, 1);
    Eigen::MatrixXd ierror(3, 1);
    Eigen::MatrixXd derror(3, 1);
    // Set sine wave parameters
    double amplitude = 0.500;
    double frequency = 1.0;
    // Set the number of points you want on the sine wave
    int numPoints = 60;
    int i;
    // Generate the sequence of points on the sine wave between A and B
    std::pair<std::vector<double>, std::vector<double>> result = generateSineWavePoints(*xpos1, *zpos1, xref, zref, amplitude, frequency, numPoints);
    std::vector<double> xref1 = result.first;
    std::vector<double> zref1 = result.second;
    std::ofstream ActualFile("t_Sinewave_trajectory_actual_1.txt");
    std::ofstream GeneratedFile("t_Sinewave_trajectory_generated_1 .txt");

    double yaw1, yawerror1, yawintegral1 = 0, prev_yaw_error1 = 0, yawspeed1, W_yaw1;
    kp = 20;
    ki = 0;
    kd = 3.55;

    //heading correction to allign in the direction of the target to avoid the escape from camera.
    //Verified no problem with heading correction
    while (live) {
        Quat q(*q1x, *q1y, *q1z, *q1w);
        EulerAngles ea = Eul_FromQuat(q, order);
        yaw1 = ea.x;
        yawerror1 = (yawref1 - yaw1);//making it impending towards the target
        yawintegral1 += yawerror1;
        yawspeed1 = kp * yawerror1 + (ki * yawintegral1 * dt) + (kd * (yawerror1 - prev_yaw_error1) / dt);
        W_yaw1 = (1 / 0.0325) * (0.126 * yawspeed1);//wheel radius 32.5 mm and lx + ly = 126 mm.
        //std::cout << "wheel speed in rpm: " << W << std::endl;
        if (abs(yawerror1) <= yaweps) {
            std::cout << "here_stop" << std::endl;
            ConnectSocket = createSocket(addr, port);
            sendWheelCommand(ConnectSocket, 0, 0, 0, 0);
            //closing the socket 
            closesocket(ConnectSocket);
            WSACleanup();
            break;
            //std::this_thread::sleep_for(std::chrono::milliseconds(400));
        }
        else {
            ConnectSocket = createSocket(addr, port);
            sendWheelCommand(ConnectSocket, -yawspeed1 / 2, yawspeed1 / 2, -yawspeed1 / 2, yawspeed1 / 2);
            //closing the socket
            closesocket(ConnectSocket);
            WSACleanup();
            //std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        prev_yaw_error1 = yawerror1;

    }
    //Sleep(1000);

    double kpx = 1.5;
    double kix = 0.05;
    double kdx = 0.55;

    double kpz = 1.5;
    double kiz = 0.05;
    double kdz = 0.55;

    double kpo = 1.5;
    double kio = 0.05;
    double kdo = 0.55;
    Eigen::MatrixXd kp1(3, 3);
    Eigen::MatrixXd ki1(3, 3);
    Eigen::MatrixXd kd1(3, 3);
    kp1 << kpz, 0, 0,
        0, kpx, 0,
        0, 0, kpo;
    ki1 << kiz, 0, 0,
        0, kix, 0,
        0, 0, kio;
    kd1 << kdz, 0, 0,
        0, kdx, 0,
        0, 0, kdo;
    ref << zref, xref, yawref1;

    for (i = 0; i < numPoints; i++) {
        ref << zref1[i], xref1[i], yawref1;
        GeneratedFile << xref1[i] << " " << zref1[i] << "\n";
        if (abs(*zpos1) - z_bound >= 0.01 || abs(*xpos1) - x_bound >= 0.01)
        {
            std::cout << "\nHorcy2 is going out of arena stopping!" << std::endl;
            ConnectSocket = createSocket(addr, port);
            sendWheelCommand(ConnectSocket, 0, 0, 0, 0);
            closesocket(ConnectSocket);
            WSACleanup();
            break;

        }
        else {
            while (live) {

                Quat q(*q1x, *q1y, *q1z, *q1w);
                ea = Eul_FromQuat(q, order);
                if (abs(ea.x - (yawref1)) >= (30 * pi / 180)) {
                    while (live) {
                        Quat q(*q1x, *q1y, *q1z, *q1w);
                        ea = Eul_FromQuat(q, order);
                        yaw1 = ea.x;
                        //std::cout << RadiansToDegrees(yaw1) << std::endl;
                        //Sleep(400);
                        yawerror1 = (yawref1 - yaw1);//making it impending towards the target
                        //std::cout << "yaw Error: " << yawerror1 << std::endl;
                        yawintegral1 += yawerror1;
                        yawspeed1 = kp * yawerror1 + (ki * yawintegral1 * dt) + (kd * (yawerror1 - prev_yaw_error1) / dt);
                        W_yaw1 = (1 / 0.0325) * (0.126 * yawspeed1);//wheel radius 32.5 mm and lx + ly = 126 mm.
                        //std::cout << "wheel speed in rpm: " << W << std::endl;
                        if (abs(yawerror1) <= yaweps1) {

                            break;

                        }
                        else {
                            ConnectSocket = createSocket(addr, port);
                            sendWheelCommand(ConnectSocket, -yawspeed1 / 2, yawspeed1 / 2, -yawspeed1 / 2, yawspeed1 / 2);
                            //closing the socket
                            closesocket(ConnectSocket);
                            WSACleanup();
                            //std::this_thread::sleep_for(std::chrono::milliseconds(100));
                        }

                        prev_yaw_error1 = yawerror1;

                    }
                }
                else {
                    ActualFile << *xpos1 << " " << *zpos1 << "\n";
                    pos << *zpos1, * xpos1, ea.x;
                    error = (ref - pos);//
                    //std::cout << "Error: " << error << std::endl;

                    integral += error;

                    perror = (kp1 * error);
                    ierror = ki1 * integral * dt;
                    derror = (kd1 * (error - preverror) / dt);
                    control = perror + ierror + derror;
                    /*Collision_control << (*U)(0, 0), (*U)(0, 1), -ea.x;
                    //std::cout<<"Control matrix:\n "<< control<< std::endl;
                   if ((*U)(0, 0) != 0 || (*U)(0, 1) != 0) {
                        W = (1 / 0.030) * (mapmat * Collision_control);
                    }*/
                    W = (1 / 0.030) * (mapmat * control);             //wheel radius 32.5 mm and lx + ly = 126 mm
                    //wheel radius 32.5 mm and lx + ly = 126 mm.
                    //std::cout << "wheel speed in rpm: " << W << std::endl;

                    if (sqrt(error(0, 0) * error(0, 0) + error(1, 0) * error(1, 0)) <= eps)
                    {
                        std::cout << "Position error below eps" << std::endl;
                        //ConnectSocket = createSocket(addr, port);
                        //sendWheelCommand(ConnectSocket, 0, 0, 0, 0);
                        //closesocket(ConnectSocket);
                        //WSACleanup();
                        break;

                    }
                    else {
                        ConnectSocket = createSocket(addr, port);
                        sendWheelCommand(ConnectSocket, W(0, 0), W(1, 0), W(2, 0), W(3, 0));
                        closesocket(ConnectSocket);
                        WSACleanup();

                    }

                    preverror = error;

                }

            }
        }
    }
    ActualFile.close();
    GeneratedFile.close();
    // errorxFile.close();
    //errorzFile.close();
    //PID to reach the final orientation
    double yaw, yawerror, yawintegral = 0, prev_yaw_error = 0, yawspeed, W_yaw;
    kp = 20;
    ki = 0;
    kd = 3.55;
    while (live) {
        if (abs(abs(*zpos1) - z_bound) <= eps || abs(abs(*xpos1) - x_bound) <= eps)
        {
            std::cout << "\nleader is going out of arena stopping!" << std::endl;
            std::cout << "zpos:" << *zpos1 << " xpos:" << *xpos1 << std::endl;
            ConnectSocket = createSocket(addr, port);
            sendWheelCommand(ConnectSocket, 0, 0, 0, 0);
            closesocket(ConnectSocket);
            WSACleanup();
            break;

        }
        else {
            Quat q(*q1x, *q1y, *q1z, *q1w);
            EulerAngles ea = Eul_FromQuat(q, order);
            yaw = ea.x;
            //std::cout << RadiansToDegrees(yaw) << std::endl;
            //Sleep(400);
            yawerror = (yawref - yaw);
            //std::cout << "yaw Error: " << yawerror << std::endl;
            //error_yawFile << yawerror << std::endl;
            yawintegral += yawerror;
            yawspeed = kp * yawerror + (ki * yawintegral * dt) + (kd * (yawerror - prev_yaw_error) / dt);
            W_yaw = (1 / 0.0325) * (0.126 * yawspeed);//wheel radius 32.5 mm and lx + ly = 126 mm.
            //std::cout << "wheel speed in rpm: " << W << std::endl;
            if (abs(yawerror) <= yaweps) {
                std::cout << "here_yaw_stop" << std::endl;
                ConnectSocket = createSocket(addr, port);
                sendWheelCommand(ConnectSocket, 0, 0, 0, 0);
                closesocket(ConnectSocket);
                WSACleanup();
                Sleep(1000);
                *flag = 1;
                break;
                //std::this_thread::sleep_for(std::chrono::milliseconds(400));
            }
            else {
                ConnectSocket = createSocket(addr, port);
                sendWheelCommand(ConnectSocket, -yawspeed / 2, yawspeed / 2, -yawspeed / 2, yawspeed / 2);
                closesocket(ConnectSocket);
                WSACleanup();
                //std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }

            prev_yaw_error = yawerror;
        }

    }
    //error_yawFile.close();

}

//Control of  the follower Robots using the modified Reynold's Flocking model
void robotControl2c(const char* addr, const int port) {
    SOCKET ConnectSocket;
    int order = EulOrdYXZr;
    const double pi = 3.1415925635897;
    Quat q(*q2x, *q2y, *q2z, *q2w);
    EulerAngles ea = Eul_FromQuat(q, order);
    bool live = true;
    double kp;
    double ki;
    double kd;
    double yawref = (0 * pi) / 180;
    double yawref1 = 0;
    double dt = 0.0045;
    Eigen::MatrixXd ref(3, 1);
    double epsa = 0.1;
    double z_bound = 2.3300;
    double x_bound = 2.000;
    double yaweps1 = (10 * pi) / 180;
    double yaweps = (10 * pi) / 180;
    Eigen::MatrixXd eps(3, 1);
    eps << epsa, epsa, yaweps1;
    Eigen::MatrixXd W(4, 1);
    Eigen::MatrixXd pos(3, 1);
    Eigen::MatrixXd mapmat(4, 3);
    mapmat << 1, -1, -0.126,
        1, 1, 0.126,
        1, 1, -0.126,
        1, -1, 0.126;
    Eigen::MatrixXd error(3, 1);
    Eigen::MatrixXd preverror(3, 1);
    preverror << 0, 0, 0;
    Eigen::MatrixXd integral(3, 1);
    integral << 0, 0, 0;
    Eigen::MatrixXd control(3, 1);
    Eigen::MatrixXd Collision_control(3, 1);
    Eigen::MatrixXd Cohesion_control(3, 1);
    Eigen::MatrixXd perror(3, 1);
    Eigen::MatrixXd ierror(3, 1);
    Eigen::MatrixXd derror(3, 1);
    double yaw1, yawerror1, yawintegral1 = 0, prev_yaw_error1 = 0, yawspeed1, W_yaw1;
    kp = 20;
    ki = 0;
    kd = 3.55;
    std::ofstream ActualFile("Sinewave_trajectory_actual_2.txt");
    //heading correction to allign in the direction of the target to avoid the escape from camera.
    //Verified no problem with heading correction
    while (live) {
        Quat q(*q2x, *q2y, *q2z, *q2w);
        ea = Eul_FromQuat(q, order);
        yaw1 = ea.x;
        yawerror1 = (yawref1 - yaw1);//making it impending towards the target
        yawintegral1 += yawerror1;
        yawspeed1 = kp * yawerror1 + (ki * yawintegral1 * dt) + (kd * (yawerror1 - prev_yaw_error1) / dt);
        W_yaw1 = (1 / 0.0325) * (0.126 * yawspeed1);//wheel radius 32.5 mm and lx + ly = 126 mm.
        if (abs(yawerror1) <= yaweps) {
            std::cout << "here_stop" << std::endl;
            ConnectSocket = createSocket(addr, port);
            sendWheelCommand(ConnectSocket, 0, 0, 0, 0);
            //closing the socket 
            closesocket(ConnectSocket);
            WSACleanup();
            break;
            //std::this_thread::sleep_for(std::chrono::milliseconds(400));
        }
        else {
            ConnectSocket = createSocket(addr, port);
            sendWheelCommand(ConnectSocket, -yawspeed1 / 2, yawspeed1 / 2, -yawspeed1 / 2, yawspeed1 / 2);
            //closing the socket
            closesocket(ConnectSocket);
            WSACleanup();
            //std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        prev_yaw_error1 = yawerror1;

    }
    double kpx = 1.5;
    double kix = 0.05;
    double kdx = 0.155;

    double kpz = 1.4;
    double kiz = 0.05;
    double kdz = 0.155;

    double kpo = 2.0;
    double kio = 0.1;
    double kdo = 0.155;
    Eigen::MatrixXd kp1(3, 3);
    Eigen::MatrixXd ki1(3, 3);
    Eigen::MatrixXd kd1(3, 3);
    kp1 << kpz, 0, 0,
        0, kpx, 0,
        0, 0, kpo;
    ki1 << kiz, 0, 0,
        0, kix, 0,
        0, 0, kio;
    kd1 << kdz, 0, 0,
        0, kdx, 0,
        0, 0, kdo;
    while (live) {
        ActualFile << *xpos2 << " " << *zpos2 << "\n";
        Quat q(*q2x, *q2y, *q2z, *q2w);
        ea = Eul_FromQuat(q, order);
        if (*flag) {
            std::cout << "\n Leader reached goal!" << std::endl;
            ConnectSocket = createSocket(addr, port);
            sendWheelCommand(ConnectSocket, 0, 0, 0, 0);
            closesocket(ConnectSocket);
            WSACleanup();
            ActualFile.close();
            break;

        }
        else {
            if (abs(abs(*zpos2) - z_bound) <= epsa || abs(abs(*xpos2) - x_bound) <= epsa)
            {
                std::cout << "\nHorcy2 is going out of arena stopping!" << std::endl;
                ConnectSocket = createSocket(addr, port);
                sendWheelCommand(ConnectSocket, 0, 0, 0, 0);
                closesocket(ConnectSocket);
                WSACleanup();
                break;

            }
            else {
                if (abs(ea.x - (yawref1)) >= (20 * pi / 180)) {
                    kp = 10;
                    ki = 0;
                    kd = 3.55;
                    while (live) {
                        Quat q(*q2x, *q2y, *q2z, *q2w);
                        ea = Eul_FromQuat(q, order);
                        yaw1 = ea.x;
                        //std::cout << RadiansToDegrees(yaw1) << std::endl;
                        //Sleep(400);
                        yawerror1 = (yawref1 - yaw1);//making it impending towards the target
                        //std::cout << "yaw Error: " << yawerror1 << std::endl;
                        yawintegral1 += yawerror1;
                        yawspeed1 = kp * yawerror1 + (ki * yawintegral1 * dt) + (kd * (yawerror1 - prev_yaw_error1) / dt);
                        W_yaw1 = (1 / 0.0325) * (0.126 * yawspeed1);//wheel radius 32.5 mm and lx + ly = 126 mm.
                        //std::cout << "wheel speed in rpm: " << W << std::endl;
                        if (abs(yawerror1) <= yaweps1) {

                            break;
                            //std::this_thread::sleep_for(std::chrono::milliseconds(400));
                        }
                        else {
                            ConnectSocket = createSocket(addr, port);
                            sendWheelCommand(ConnectSocket, -yawspeed1 / 2, yawspeed1 / 2, -yawspeed1 / 2, yawspeed1 / 2);
                            //closing the socket
                            closesocket(ConnectSocket);
                            WSACleanup();
                        }

                        prev_yaw_error1 = yawerror1;

                    }
                }
                else {
                    Collision_control << (*U)(1, 0), (*U)(1, 1), -ea.x;
                    Cohesion_control << (*F_Ch)(1, 0), (*F_Ch)(1, 1), -ea.x;
                    if ((*U)(1, 0) == 0 && (*U)(1, 1) == 0 && (*F_Ch)(1, 0) == 0 && (*F_Ch)(1, 1) == 0)
                    {
                        std::cout << "\nHorcy2 Out of sensing radius" << std::endl;
                        ConnectSocket = createSocket(addr, port);
                        sendWheelCommand(ConnectSocket, 0, 0, 0, 0);
                        closesocket(ConnectSocket);
                        WSACleanup();
                    }
                    else {
                        error = (Cohesion_control + Collision_control) * dt;
                        integral += error;
                        perror = (kp1 * error);
                        ierror = ki1 * integral * dt;
                        derror = (kd1 * (error - preverror) / dt);
                        control = perror + ierror + derror;
                        //std::cout << control << std::endl;
                        /*if ((*U)(1, 0) != 0 || (*U)(1, 1) != 0) {
                            W = (1 / 0.030) * (mapmat * (Collision_control));
                        }*/

                        W = (1 / 0.030) * (mapmat * (control));                //wheel radius 32.5 mm and lx + ly = 126 mm.
                        ConnectSocket = createSocket(addr, port);
                        sendWheelCommand(ConnectSocket, W(0, 0), W(1, 0), W(2, 0), W(3, 0));
                        closesocket(ConnectSocket);
                        WSACleanup();
                        preverror = error;

                    }
                }
            }
        }
    }




}
void robotControl3c(const char* addr, const int port) {
    SOCKET ConnectSocket;
    int order = EulOrdYXZr;
    const double pi = 3.1415925635897;
    Quat q(*q3x, *q3y, *q3z, *q3w);
    EulerAngles ea = Eul_FromQuat(q, order);
    bool live = true;
    double kp;
    double ki;
    double kd;
    double yawref = (0 * pi) / 180;
    double yawref1 = 0;
    double dt = 0.0045;
    Eigen::MatrixXd ref(3, 1);
    double epsa = 0.1;
    double xeps = 0.1;
    double zeps = 0.1;
    double z_bound = 2.3300;
    double x_bound = 2.000;
    double yaweps1 = (10 * pi) / 180;
    double yaweps = (10 * pi) / 180;
    Eigen::MatrixXd eps(3, 1);
    eps << zeps, xeps, yaweps1;
    Eigen::MatrixXd W(4, 1);
    Eigen::MatrixXd pos(3, 1);
    Eigen::MatrixXd mapmat(4, 3);
    mapmat << 1, -1, -0.126,
        1, 1, 0.126,
        1, 1, -0.126,
        1, -1, 0.126;
    Eigen::MatrixXd error(3, 1);
    Eigen::MatrixXd preverror(3, 1);
    preverror << 0, 0, 0;
    Eigen::MatrixXd integral(3, 1);
    integral << 0, 0, 0;
    Eigen::MatrixXd control(3, 1);
    Eigen::MatrixXd Collision_control(3, 1);
    Eigen::MatrixXd Cohesion_control(3, 1);
    Eigen::MatrixXd perror(3, 1);
    Eigen::MatrixXd ierror(3, 1);
    Eigen::MatrixXd derror(3, 1);
    double yaw1, yawerror1, yawintegral1 = 0, prev_yaw_error1 = 0, yawspeed1, W_yaw1;
    kp = 20;
    ki = 0;
    kd = 3.55;
    std::ofstream ActualFile("Sinewave_trajectory_actual_3.txt");
    //heading correction to allign in the direction of the target to avoid the escape from camera.
    //Verified no problem with heading correction
    while (live) {
        Quat q(*q3x, *q3y, *q3z, *q3w);
        ea = Eul_FromQuat(q, order);
        yaw1 = ea.x;
        yawerror1 = (yawref1 - yaw1);//making it impending towards the target
        yawintegral1 += yawerror1;
        yawspeed1 = kp * yawerror1 + (ki * yawintegral1 * dt) + (kd * (yawerror1 - prev_yaw_error1) / dt);
        W_yaw1 = (1 / 0.0325) * (0.126 * yawspeed1);//wheel radius 32.5 mm and lx + ly = 126 mm.
        if (abs(yawerror1) <= yaweps) {
            std::cout << "here_stop" << std::endl;
            ConnectSocket = createSocket(addr, port);
            sendWheelCommand(ConnectSocket, 0, 0, 0, 0);
            //closing the socket 
            closesocket(ConnectSocket);
            WSACleanup();
            break;
        }
        else {
            ConnectSocket = createSocket(addr, port);
            sendWheelCommand(ConnectSocket, -yawspeed1 / 2, yawspeed1 / 2, -yawspeed1 / 2, yawspeed1 / 2);
            //closing the socket
            closesocket(ConnectSocket);
            WSACleanup();
        }

        prev_yaw_error1 = yawerror1;

    }
    double kpx = 1.5;
    double kix = 0.05;
    double kdx = 0.155;

    double kpz = 1.4;
    double kiz = 0.05;
    double kdz = 0.155;

    double kpo = 1.3;
    double kio = 0.05;
    double kdo = 0.155;
    Eigen::MatrixXd kp1(3, 3);
    Eigen::MatrixXd ki1(3, 3);
    Eigen::MatrixXd kd1(3, 3);
    kp1 << kpz, 0, 0,
        0, kpx, 0,
        0, 0, kpo;
    ki1 << kiz, 0, 0,
        0, kix, 0,
        0, 0, kio;
    kd1 << kdz, 0, 0,
        0, kdx, 0,
        0, 0, kdo;
    while (live) {
        ActualFile << *xpos3 << " " << *zpos3 << "\n";
        Quat q(*q3x, *q3y, *q3z, *q3w);
        ea = Eul_FromQuat(q, order);
        if (*flag) {
            std::cout << "\n Leader reached goal!" << std::endl;
            ConnectSocket = createSocket(addr, port);
            sendWheelCommand(ConnectSocket, 0, 0, 0, 0);
            closesocket(ConnectSocket);
            WSACleanup();
            ActualFile.close();
            break;

        }
        else {
            if (abs(abs(*zpos3) - z_bound) <= zeps || abs(abs(*xpos3) - x_bound) <= xeps)
            {
                std::cout << "\nHorcy3 is going out of arena stopping!" << std::endl;
                ConnectSocket = createSocket(addr, port);
                sendWheelCommand(ConnectSocket, 0, 0, 0, 0);
                closesocket(ConnectSocket);
                WSACleanup();
                break;

            }
            else {
                if (abs(ea.x - (yawref1)) >= (20 * pi / 180)) {
                    kp = 10;
                    ki = 0;
                    kd = 3.55;
                    while (live) {
                        Quat q(*q3x, *q3y, *q3z, *q3w);
                        ea = Eul_FromQuat(q, order);
                        yaw1 = ea.x;
                        yawerror1 = (yawref1 - yaw1);//making it impending towards the target
                        yawintegral1 += yawerror1;
                        yawspeed1 = kp * yawerror1 + (ki * yawintegral1 * dt) + (kd * (yawerror1 - prev_yaw_error1) / dt);
                        W_yaw1 = (1 / 0.0325) * (0.126 * yawspeed1);//wheel radius 32.5 mm and lx + ly = 126 mm.
                        if (abs(yawerror1) <= yaweps1) {

                            break;
                            //std::this_thread::sleep_for(std::chrono::milliseconds(400));
                        }
                        else {
                            ConnectSocket = createSocket(addr, port);
                            sendWheelCommand(ConnectSocket, -yawspeed1 / 2, yawspeed1 / 2, -yawspeed1 / 2, yawspeed1 / 2);
                            //closing the socket
                            closesocket(ConnectSocket);
                            WSACleanup();
                        }

                        prev_yaw_error1 = yawerror1;

                    }
                }
                else {

                    Collision_control << (*U)(2, 0), (*U)(2, 1), -ea.x;
                    Cohesion_control << (*F_Ch)(2, 0), (*F_Ch)(2, 1), -ea.x;
                    if ((*U)(2, 0) == 0 && (*U)(2, 1) == 0 && (*F_Ch)(2, 0) == 0 && (*F_Ch)(2, 1) == 0)
                    {
                        std::cout << "\nHorcy3 Out of sensing radius" << std::endl;
                        ConnectSocket = createSocket(addr, port);
                        sendWheelCommand(ConnectSocket, 0, 0, 0, 0);
                        closesocket(ConnectSocket);
                        WSACleanup();
                    }
                    else {
                        error = (Cohesion_control + Collision_control) * dt;
                        integral += error;
                        perror = (kp1 * error);
                        ierror = ki1 * integral * dt;
                        derror = (kd1 * (error - preverror) / dt);
                        control = perror + ierror + derror;
                        /*if ((*U)(2, 0) != 0 || (*U)(2, 1) != 0) {
                            W = (1 / 0.030) * (mapmat * (Collision_control));
                        }*/

                        W = (1 / 0.030) * (mapmat * (control));                //wheel radius 32.5 mm and lx + ly = 126 mm.
                        ConnectSocket = createSocket(addr, port);
                        sendWheelCommand(ConnectSocket, W(0, 0), W(1, 0), W(2, 0), W(3, 0));
                        closesocket(ConnectSocket);
                        WSACleanup();

                        preverror = error;
                    }
                }
            }
        }
    }




}
void robotControl4c(const char* addr, const int port) {
    SOCKET ConnectSocket;
    int order = EulOrdYXZr;
    const double pi = 3.1415925635897;
    Quat q(*q4x, *q4y, *q4z, *q4w);
    EulerAngles ea = Eul_FromQuat(q, order);
    bool live = true;
    double kp;
    double ki;
    double kd;
    double yawref = (0 * pi) / 180;
    double yawref1 = 0;
    double dt = 0.0045;
    Eigen::MatrixXd ref(3, 1);
    double epsa = 0.1;
    double xeps = 0.1;
    double zeps = 0.1;
    double z_bound = 2.3300;
    double x_bound = 2.000;
    double yaweps1 = (10 * pi) / 180;
    double yaweps = (10 * pi) / 180;
    Eigen::MatrixXd eps(3, 1);
    eps << zeps, xeps, yaweps1;
    Eigen::MatrixXd W(4, 1);
    Eigen::MatrixXd pos(3, 1);
    Eigen::MatrixXd mapmat(4, 3);
    mapmat << 1, -1, -0.126,
        1, 1, 0.126,
        1, 1, -0.126,
        1, -1, 0.126;
    Eigen::MatrixXd error(3, 1);
    Eigen::MatrixXd preverror(3, 1);
    preverror << 0, 0, 0;
    Eigen::MatrixXd integral(3, 1);
    integral << 0, 0, 0;
    Eigen::MatrixXd control(3, 1);
    Eigen::MatrixXd Collision_control(3, 1);
    Eigen::MatrixXd Cohesion_control(3, 1);
    Eigen::MatrixXd perror(3, 1);
    Eigen::MatrixXd ierror(3, 1);
    Eigen::MatrixXd derror(3, 1);
    double yaw1, yawerror1, yawintegral1 = 0, prev_yaw_error1 = 0, yawspeed1, W_yaw1;
    kp = 20;
    ki = 0;
    kd = 3.55;
    std::ofstream ActualFile("Sinewave_trajectory_actual_4.txt");
    //heading correction to allign in the direction of the target to avoid the escape from camera.
    //Verified no problem with heading correction
    while (live) {
        Quat q(*q4x, *q4y, *q4z, *q4w);
        ea = Eul_FromQuat(q, order);
        yaw1 = ea.x;
        yawerror1 = (yawref1 - yaw1);//making it impending towards the target
        yawintegral1 += yawerror1;
        yawspeed1 = kp * yawerror1 + (ki * yawintegral1 * dt) + (kd * (yawerror1 - prev_yaw_error1) / dt);
        W_yaw1 = (1 / 0.0325) * (0.126 * yawspeed1);//wheel radius 32.5 mm and lx + ly = 126 mm.
        if (abs(yawerror1) <= yaweps) {
            std::cout << "here_stop" << std::endl;
            ConnectSocket = createSocket(addr, port);
            sendWheelCommand(ConnectSocket, 0, 0, 0, 0);
            //closing the socket 
            closesocket(ConnectSocket);
            WSACleanup();
            break;
        }
        else {
            ConnectSocket = createSocket(addr, port);
            sendWheelCommand(ConnectSocket, -yawspeed1 / 2, yawspeed1 / 2, -yawspeed1 / 2, yawspeed1 / 2);
            //closing the socket
            closesocket(ConnectSocket);
            WSACleanup();
        }

        prev_yaw_error1 = yawerror1;

    }
    double kpx = 1.5;
    double kix = 0.05;
    double kdx = 0.155;

    double kpz = 1.4;
    double kiz = 0.05;
    double kdz = 0.155;

    double kpo = 1.3;
    double kio = 0.05;
    double kdo = 0.155;
    Eigen::MatrixXd kp1(3, 3);
    Eigen::MatrixXd ki1(3, 3);
    Eigen::MatrixXd kd1(3, 3);
    kp1 << kpz, 0, 0,
        0, kpx, 0,
        0, 0, kpo;
    ki1 << kiz, 0, 0,
        0, kix, 0,
        0, 0, kio;
    kd1 << kdz, 0, 0,
        0, kdx, 0,
        0, 0, kdo;
    while (live) {
        ActualFile << *xpos4 << " " << *zpos4 << "\n";
        Quat q(*q4x, *q4y, *q4z, *q4w);
        ea = Eul_FromQuat(q, order);
        if (*flag) {
            std::cout << "\n Leader reached goal!" << std::endl;
            ConnectSocket = createSocket(addr, port);
            sendWheelCommand(ConnectSocket, 0, 0, 0, 0);
            closesocket(ConnectSocket);
            WSACleanup();
            ActualFile.close();
            break;

        }
        else {
            if (abs(abs(*zpos4) - z_bound) <= zeps || abs(abs(*xpos4) - x_bound) <= xeps)
            {
                std::cout << "\nHorcy3 is going out of arena stopping!" << std::endl;
                ConnectSocket = createSocket(addr, port);
                sendWheelCommand(ConnectSocket, 0, 0, 0, 0);
                closesocket(ConnectSocket);
                WSACleanup();
                break;

            }
            else {
                if (abs(ea.x - (yawref1)) >= (20 * pi / 180)) {
                    kp = 10;
                    ki = 0;
                    kd = 3.55;
                    while (live) {
                        Quat q(*q4x, *q4y, *q4z, *q4w);
                        ea = Eul_FromQuat(q, order);
                        yaw1 = ea.x;
                        yawerror1 = (yawref1 - yaw1);//making it impending towards the target
                        yawintegral1 += yawerror1;
                        yawspeed1 = kp * yawerror1 + (ki * yawintegral1 * dt) + (kd * (yawerror1 - prev_yaw_error1) / dt);
                        W_yaw1 = (1 / 0.0325) * (0.126 * yawspeed1);//wheel radius 32.5 mm and lx + ly = 126 mm.
                        if (abs(yawerror1) <= yaweps1) {

                            break;
                            //std::this_thread::sleep_for(std::chrono::milliseconds(400));
                        }
                        else {
                            ConnectSocket = createSocket(addr, port);
                            sendWheelCommand(ConnectSocket, -yawspeed1 / 2, yawspeed1 / 2, -yawspeed1 / 2, yawspeed1 / 2);
                            //closing the socket
                            closesocket(ConnectSocket);
                            WSACleanup();
                        }

                        prev_yaw_error1 = yawerror1;

                    }
                }
                else {
                    Collision_control << (*U)(3, 0), (*U)(3, 1), -ea.x;
                    Cohesion_control << (*F_Ch)(3, 0), (*F_Ch)(3, 1), -ea.x;
                    if ((*U)(3, 0) == 0 && (*U)(3, 1) == 0 && (*F_Ch)(3, 0) == 0 && (*F_Ch)(3, 1) == 0)
                    {
                        std::cout << "\nHorcy3 Out of sensing radius" << std::endl;
                        ConnectSocket = createSocket(addr, port);
                        sendWheelCommand(ConnectSocket, 0, 0, 0, 0);
                        closesocket(ConnectSocket);
                        WSACleanup();
                    }
                    else {
                        error = (Cohesion_control + Collision_control) * dt;
                        integral += error;
                        perror = (kp1 * error);
                        ierror = ki1 * integral * dt;
                        derror = (kd1 * (error - preverror) / dt);
                        control = perror + ierror + derror;
                        /*if ((*U)(3, 0) != 0 || (*U)(3, 1) != 0) {
                            W = (1 / 0.030) * (mapmat * (Collision_control));
                        }*/
                        W = (1 / 0.030) * (mapmat * (control));                //wheel radius 32.5 mm and lx + ly = 126 mm.
                        ConnectSocket = createSocket(addr, port);
                        sendWheelCommand(ConnectSocket, W(0, 0), W(1, 0), W(2, 0), W(3, 0));
                        closesocket(ConnectSocket);
                        WSACleanup();

                        preverror = error;
                    }
                }
            }
        }
    }




}
void robotControl5c(const char* addr, const int port) {
    SOCKET ConnectSocket;
    int order = EulOrdYXZr;
    const double pi = 3.1415925635897;
    Quat q(*q5x, *q5y, *q5z, *q5w);
    EulerAngles ea = Eul_FromQuat(q, order);
    bool live = true;
    double kp;
    double ki;
    double kd;
    double yawref = (0 * pi) / 180;
    double yawref1 = 0;
    double dt = 0.0045;
    Eigen::MatrixXd ref(3, 1);
    double epsa = 0.1;
    double xeps = 0.1;
    double zeps = 0.1;
    double z_bound = 2.3300;
    double x_bound = 2.000;
    double yaweps1 = (10 * pi) / 180;
    double yaweps = (10 * pi) / 180;
    Eigen::MatrixXd eps(3, 1);
    eps << zeps, xeps, yaweps1;
    Eigen::MatrixXd W(4, 1);
    Eigen::MatrixXd pos(3, 1);
    Eigen::MatrixXd mapmat(4, 3);
    mapmat << 1, -1, -0.126,
        1, 1, 0.126,
        1, 1, -0.126,
        1, -1, 0.126;
    Eigen::MatrixXd error(3, 1);
    Eigen::MatrixXd preverror(3, 1);
    preverror << 0, 0, 0;
    Eigen::MatrixXd integral(3, 1);
    integral << 0, 0, 0;
    Eigen::MatrixXd control(3, 1);
    Eigen::MatrixXd Collision_control(3, 1);
    Eigen::MatrixXd Cohesion_control(3, 1);
    Eigen::MatrixXd perror(3, 1);
    Eigen::MatrixXd ierror(3, 1);
    Eigen::MatrixXd derror(3, 1);
    double yaw1, yawerror1, yawintegral1 = 0, prev_yaw_error1 = 0, yawspeed1, W_yaw1;
    kp = 20;
    ki = 0;
    kd = 3.55;
    std::ofstream ActualFile("Sinewave_trajectory_actual_5.txt");
    //heading correction to allign in the direction of the target to avoid the escape from camera.
    //Verified no problem with heading correction
    while (live) {
        Quat q(*q5x, *q5y, *q5z, *q5w);
        ea = Eul_FromQuat(q, order);
        yaw1 = ea.x;
        yawerror1 = (yawref1 - yaw1);//making it impending towards the target
        yawintegral1 += yawerror1;
        yawspeed1 = kp * yawerror1 + (ki * yawintegral1 * dt) + (kd * (yawerror1 - prev_yaw_error1) / dt);
        W_yaw1 = (1 / 0.0325) * (0.126 * yawspeed1);//wheel radius 32.5 mm and lx + ly = 126 mm.
        if (abs(yawerror1) <= yaweps) {
            std::cout << "here_stop" << std::endl;
            ConnectSocket = createSocket(addr, port);
            sendWheelCommand(ConnectSocket, 0, 0, 0, 0);
            //closing the socket 
            closesocket(ConnectSocket);
            WSACleanup();
            break;
        }
        else {
            ConnectSocket = createSocket(addr, port);
            sendWheelCommand(ConnectSocket, -yawspeed1 / 2, yawspeed1 / 2, -yawspeed1 / 2, yawspeed1 / 2);
            //closing the socket
            closesocket(ConnectSocket);
            WSACleanup();
        }

        prev_yaw_error1 = yawerror1;

    }
    double kpx = 1.5;
    double kix = 0.05;
    double kdx = 0.155;

    double kpz = 1.4;
    double kiz = 0.05;
    double kdz = 0.155;

    double kpo = 1.3;
    double kio = 0.05;
    double kdo = 0.155;
    Eigen::MatrixXd kp1(3, 3);
    Eigen::MatrixXd ki1(3, 3);
    Eigen::MatrixXd kd1(3, 3);
    kp1 << kpz, 0, 0,
        0, kpx, 0,
        0, 0, kpo;
    ki1 << kiz, 0, 0,
        0, kix, 0,
        0, 0, kio;
    kd1 << kdz, 0, 0,
        0, kdx, 0,
        0, 0, kdo;
    while (live) {
        ActualFile << *xpos5 << " " << *zpos5 << "\n";
        Quat q(*q5x, *q5y, *q5z, *q5w);
        ea = Eul_FromQuat(q, order);
        if (*flag) {
            std::cout << "\n Leader reached goal!" << std::endl;
            ConnectSocket = createSocket(addr, port);
            sendWheelCommand(ConnectSocket, 0, 0, 0, 0);
            closesocket(ConnectSocket);
            WSACleanup();
            ActualFile.close();
            break;

        }
        else {
            if (abs(abs(*zpos5) - z_bound) <= zeps || abs(abs(*xpos5) - x_bound) <= xeps)
            {
                std::cout << "\nHorcy5 is going out of arena stopping!" << std::endl;
                ConnectSocket = createSocket(addr, port);
                sendWheelCommand(ConnectSocket, 0, 0, 0, 0);
                closesocket(ConnectSocket);
                WSACleanup();
                break;

            }
            else {
                if (abs(ea.x - (yawref1)) >= (20 * pi / 180)) {
                    kp = 10;
                    ki = 0;
                    kd = 3.55;
                    while (live) {
                        Quat q(*q5x, *q5y, *q5z, *q5w);
                        ea = Eul_FromQuat(q, order);
                        yaw1 = ea.x;
                        yawerror1 = (yawref1 - yaw1);//making it impending towards the target
                        yawintegral1 += yawerror1;
                        yawspeed1 = kp * yawerror1 + (ki * yawintegral1 * dt) + (kd * (yawerror1 - prev_yaw_error1) / dt);
                        W_yaw1 = (1 / 0.0325) * (0.126 * yawspeed1);//wheel radius 32.5 mm and lx + ly = 126 mm.
                        if (abs(yawerror1) <= yaweps1) {

                            break;
                            //std::this_thread::sleep_for(std::chrono::milliseconds(400));
                        }
                        else {
                            ConnectSocket = createSocket(addr, port);
                            sendWheelCommand(ConnectSocket, -yawspeed1 / 2, yawspeed1 / 2, -yawspeed1 / 2, yawspeed1 / 2);
                            //closing the socket
                            closesocket(ConnectSocket);
                            WSACleanup();
                        }

                        prev_yaw_error1 = yawerror1;

                    }
                }
                else {
                    Collision_control << (*U)(4, 0), (*U)(4, 1), -ea.x;
                    Cohesion_control << (*F_Ch)(4, 0), (*F_Ch)(4, 1), -ea.x;
                    if ((*U)(4, 0) == 0 && (*U)(4, 1) == 0 && (*F_Ch)(4, 0) == 0 && (*F_Ch)(4, 1) == 0)
                    {
                        std::cout << "\nHorcy3 Out of sensing radius" << std::endl;
                        ConnectSocket = createSocket(addr, port);
                        sendWheelCommand(ConnectSocket, 0, 0, 0, 0);
                        closesocket(ConnectSocket);
                        WSACleanup();
                    }
                    else {
                        error = (Cohesion_control + Collision_control) * dt;
                        integral += error;
                        perror = (kp1 * error);
                        ierror = ki1 * integral * dt;
                        derror = (kd1 * (error - preverror) / dt);
                        control = perror + ierror + derror;
                        W = (1 / 0.030) * (mapmat * (control));                //wheel radius 32.5 mm and lx + ly = 126 mm.
                        ConnectSocket = createSocket(addr, port);
                        sendWheelCommand(ConnectSocket, W(0, 0), W(1, 0), W(2, 0), W(3, 0));
                        closesocket(ConnectSocket);
                        WSACleanup();

                        preverror = error;
                    }
                }
            }
        }
    }




}
void robotControl6c(const char* addr, const int port) {
    SOCKET ConnectSocket;
    int order = EulOrdYXZr;
    const double pi = 3.1415925635897;
    Quat q(*q6x, *q6y, *q6z, *q6w);
    EulerAngles ea = Eul_FromQuat(q, order);
    bool live = true;
    double kp;
    double ki;
    double kd;
    double yawref = (0 * pi) / 180;
    double yawref1 = 0;
    double dt = 0.0045;
    Eigen::MatrixXd ref(3, 1);
    double epsa = 0.1;
    double xeps = 0.1;
    double zeps = 0.1;
    double z_bound = 2.3300;
    double x_bound = 2.000;
    double yaweps1 = (10 * pi) / 180;
    double yaweps = (10 * pi) / 180;
    Eigen::MatrixXd eps(3, 1);
    eps << zeps, xeps, yaweps1;
    Eigen::MatrixXd W(4, 1);
    Eigen::MatrixXd pos(3, 1);
    Eigen::MatrixXd mapmat(4, 3);
    mapmat << 1, -1, -0.126,
        1, 1, 0.126,
        1, 1, -0.126,
        1, -1, 0.126;
    Eigen::MatrixXd error(3, 1);
    Eigen::MatrixXd preverror(3, 1);
    preverror << 0, 0, 0;
    Eigen::MatrixXd integral(3, 1);
    integral << 0, 0, 0;
    Eigen::MatrixXd control(3, 1);
    Eigen::MatrixXd Collision_control(3, 1);
    Eigen::MatrixXd Cohesion_control(3, 1);
    Eigen::MatrixXd perror(3, 1);
    Eigen::MatrixXd ierror(3, 1);
    Eigen::MatrixXd derror(3, 1);
    double yaw1, yawerror1, yawintegral1 = 0, prev_yaw_error1 = 0, yawspeed1, W_yaw1;
    kp = 20;
    ki = 0;
    kd = 3.55;
    std::ofstream ActualFile("Sinewave_trajectory_actual_6.txt");
    //heading correction to allign in the direction of the target to avoid the escape from camera.
    //Verified no problem with heading correction
    while (live) {
        Quat q(*q6x, *q6y, *q6z, *q6w);
        ea = Eul_FromQuat(q, order);
        yaw1 = ea.x;
        yawerror1 = (yawref1 - yaw1);//making it impending towards the target
        yawintegral1 += yawerror1;
        yawspeed1 = kp * yawerror1 + (ki * yawintegral1 * dt) + (kd * (yawerror1 - prev_yaw_error1) / dt);
        W_yaw1 = (1 / 0.0325) * (0.126 * yawspeed1);//wheel radius 32.5 mm and lx + ly = 126 mm.
        if (abs(yawerror1) <= yaweps) {
            std::cout << "here_stop" << std::endl;
            ConnectSocket = createSocket(addr, port);
            sendWheelCommand(ConnectSocket, 0, 0, 0, 0);
            //closing the socket 
            closesocket(ConnectSocket);
            WSACleanup();
            break;
        }
        else {
            ConnectSocket = createSocket(addr, port);
            sendWheelCommand(ConnectSocket, -yawspeed1 / 2, yawspeed1 / 2, -yawspeed1 / 2, yawspeed1 / 2);
            //closing the socket
            closesocket(ConnectSocket);
            WSACleanup();
        }

        prev_yaw_error1 = yawerror1;

    }
    double kpx = 1.5;
    double kix = 0.05;
    double kdx = 0.155;

    double kpz = 1.4;
    double kiz = 0.05;
    double kdz = 0.155;

    double kpo = 1.3;
    double kio = 0.05;
    double kdo = 0.155;
    Eigen::MatrixXd kp1(3, 3);
    Eigen::MatrixXd ki1(3, 3);
    Eigen::MatrixXd kd1(3, 3);
    kp1 << kpz, 0, 0,
        0, kpx, 0,
        0, 0, kpo;
    ki1 << kiz, 0, 0,
        0, kix, 0,
        0, 0, kio;
    kd1 << kdz, 0, 0,
        0, kdx, 0,
        0, 0, kdo;
    while (live) {
        ActualFile << *xpos6 << " " << *zpos6 << "\n";
        Quat q(*q6x, *q6y, *q6z, *q6w);
        ea = Eul_FromQuat(q, order);
        if (*flag) {
            std::cout << "\n Leader reached goal!" << std::endl;
            ConnectSocket = createSocket(addr, port);
            sendWheelCommand(ConnectSocket, 0, 0, 0, 0);
            closesocket(ConnectSocket);
            WSACleanup();
            ActualFile.close();
            break;

        }
        else {
            if (abs(abs(*zpos6) - z_bound) <= zeps || abs(abs(*xpos6) - x_bound) <= xeps)
            {
                std::cout << "\nHorcy6 is going out of arena stopping!" << std::endl;
                ConnectSocket = createSocket(addr, port);
                sendWheelCommand(ConnectSocket, 0, 0, 0, 0);
                closesocket(ConnectSocket);
                WSACleanup();
                break;

            }
            else {
                if (abs(ea.x - (yawref1)) >= (20 * pi / 180)) {
                    kp = 10;
                    ki = 0;
                    kd = 3.55;
                    while (live) {
                        Quat q(*q6x, *q6y, *q6z, *q6w);
                        ea = Eul_FromQuat(q, order);
                        yaw1 = ea.x;
                        yawerror1 = (yawref1 - yaw1);//making it impending towards the target
                        yawintegral1 += yawerror1;
                        yawspeed1 = kp * yawerror1 + (ki * yawintegral1 * dt) + (kd * (yawerror1 - prev_yaw_error1) / dt);
                        W_yaw1 = (1 / 0.0325) * (0.126 * yawspeed1);//wheel radius 32.5 mm and lx + ly = 126 mm.
                        if (abs(yawerror1) <= yaweps1) {

                            break;
                            //std::this_thread::sleep_for(std::chrono::milliseconds(400));
                        }
                        else {
                            ConnectSocket = createSocket(addr, port);
                            sendWheelCommand(ConnectSocket, -yawspeed1 / 2, yawspeed1 / 2, -yawspeed1 / 2, yawspeed1 / 2);
                            //closing the socket
                            closesocket(ConnectSocket);
                            WSACleanup();
                        }

                        prev_yaw_error1 = yawerror1;

                    }
                }
                else {
                    Collision_control << (*U)(5, 0), (*U)(5, 1), -ea.x;
                    Cohesion_control << (*F_Ch)(5, 0), (*F_Ch)(5, 1), -ea.x;
                    if ((*U)(5, 0) == 0 && (*U)(5, 1) == 0 && (*F_Ch)(5, 0) == 0 && (*F_Ch)(5, 1) == 0)
                    {
                        std::cout << "\nHorcy6 Out of sensing radius" << std::endl;
                        ConnectSocket = createSocket(addr, port);
                        sendWheelCommand(ConnectSocket, 0, 0, 0, 0);
                        closesocket(ConnectSocket);
                        WSACleanup();
                    }
                    else {
                        error = (Cohesion_control + Collision_control) * dt;
                        integral += error;
                        perror = (kp1 * error);
                        ierror = ki1 * integral * dt;
                        derror = (kd1 * (error - preverror) / dt);
                        control = perror + ierror + derror;
                        W = (1 / 0.030) * (mapmat * (control));                //wheel radius 32.5 mm and lx + ly = 126 mm.
                        ConnectSocket = createSocket(addr, port);
                        sendWheelCommand(ConnectSocket, W(0, 0), W(1, 0), W(2, 0), W(3, 0));
                        closesocket(ConnectSocket);
                        WSACleanup();

                        preverror = error;
                    }
                }
            }
        }
    }




}

//Streaming data from the motive using Natnet SDK
void cameraTracking(int argc, char* argv[]) {
    unique_lock<mutex> cameralock(camera);
    // print version info
    unsigned char ver[4];
    NatNet_GetVersion(ver);
    printf("NatNet Sample Client (NatNet ver. %d.%d.%d.%d)\n", ver[0], ver[1], ver[2], ver[3]);

    // Install logging callback
    NatNet_SetLogCallback(MessageHandler);

    // create NatNet client
    g_pClient = new NatNetClient();

    // set the frame callback handler
    g_pClient->SetFrameReceivedCallback(DataHandler, g_pClient);	// this function will receive data from the server


    // If no arguments were specified on the command line,
    // attempt to discover servers on the local network.
    if (argc == 1)
    {

        {
            // An example of synchronous server discovery.
#if 0
            const unsigned int kDiscoveryWaitTimeMillisec = 5 * 1000; // Wait 5 seconds for responses.
            const int kMaxDescriptions = 10; // Get info for, at most, the first 10 servers to respond.
            sNatNetDiscoveredServer servers[kMaxDescriptions];
            int actualNumDescriptions = kMaxDescriptions;
            NatNet_BroadcastServerDiscovery(servers, &actualNumDescriptions);

            if (actualNumDescriptions < kMaxDescriptions)
            {
                // If this happens, more servers responded than the array was able to store.
            }
#endif

            // Do asynchronous server discovery.
            printf("Looking for servers on the local network.\n");
            printf("Press the number key that corresponds to any discovered server to connect to that server.\n");
            printf("Press Q at any time to quit.\n\n");

            NatNetDiscoveryHandle discovery;
            NatNet_CreateAsyncServerDiscovery(&discovery, ServerDiscoveredCallback);
            //while (int c = _getch())
            while (const int c = 49)
            {
                if (c >= '1' && c <= '9')
                {
                    const size_t serverIndex = c - '1';
                    if (serverIndex < g_discoveredServers.size())
                    {
                        const sNatNetDiscoveredServer& discoveredServer = g_discoveredServers[serverIndex];

                        if (discoveredServer.serverDescription.bConnectionInfoValid)
                        {
                            // Build the connection parameters.
#ifdef _WIN32
                            _snprintf_s(
#endif
                                g_discoveredMulticastGroupAddr, sizeof g_discoveredMulticastGroupAddr,
                                "%" PRIu8 ".%" PRIu8".%" PRIu8".%" PRIu8"",
                                discoveredServer.serverDescription.ConnectionMulticastAddress[0],
                                discoveredServer.serverDescription.ConnectionMulticastAddress[1],
                                discoveredServer.serverDescription.ConnectionMulticastAddress[2],
                                discoveredServer.serverDescription.ConnectionMulticastAddress[3]
                            );

                            g_connectParams.connectionType = discoveredServer.serverDescription.ConnectionMulticast ? ConnectionType_Multicast : ConnectionType_Unicast;
                            g_connectParams.serverCommandPort = discoveredServer.serverCommandPort;
                            g_connectParams.serverDataPort = discoveredServer.serverDescription.ConnectionDataPort;
                            g_connectParams.serverAddress = discoveredServer.serverAddress;
                            g_connectParams.localAddress = discoveredServer.localAddress;
                            g_connectParams.multicastAddress = g_discoveredMulticastGroupAddr;
                        }
                        else
                        {
                            // We're missing some info because it's a legacy server.
                            // Guess the defaults and make a best effort attempt to connect.
                            g_connectParams.connectionType = kDefaultConnectionType;
                            g_connectParams.serverCommandPort = discoveredServer.serverCommandPort;
                            g_connectParams.serverDataPort = 0;
                            g_connectParams.serverAddress = discoveredServer.serverAddress;
                            g_connectParams.localAddress = discoveredServer.localAddress;
                            g_connectParams.multicastAddress = NULL;
                        }

                        break;
                    }
                }
                else if (c == 'q')
                {
                    return;
                }
            }

            NatNet_FreeAsyncServerDiscovery(discovery);
        }
    }
    else
    {
        g_connectParams.connectionType = kDefaultConnectionType;

        if (argc >= 2)
        {
            g_connectParams.serverAddress = argv[1];
        }

        if (argc >= 3)
        {
            g_connectParams.localAddress = argv[2];
        }
    }

    int iResult;

    // Connect to Motive
    iResult = ConnectClient();
    if (iResult != ErrorCode_OK)
    {
        printf("Error initializing client. See log for details. Exiting.\n");
        return; // Exit the function without returning an error code
    }
    else
    {
        printf("Client initialized and ready.\n");
    }


    // Send/receive test request
    void* response;
    int nBytes;
    printf("[SampleClient] Sending Test Request\n");
    iResult = g_pClient->SendMessageAndWait("TestRequest", &response, &nBytes);
    if (iResult == ErrorCode_OK)
    {
        printf("[SampleClient] Received: %s\n", (char*)response);
    }

    // Retrieve Data Descriptions from Motive
    printf("\n\n[SampleClient] Requesting Data Descriptions...\n");
    sDataDescriptions* pDataDefs = NULL;
    iResult = g_pClient->GetDataDescriptionList(&pDataDefs);
    if (iResult != ErrorCode_OK || pDataDefs == NULL)
    {
        printf("[SampleClient] Unable to retrieve Data Descriptions.\n");
    }
    else
    {
        printf("[SampleClient] Received %d Data Descriptions:\n", pDataDefs->nDataDescriptions);
        for (int i = 0; i < pDataDefs->nDataDescriptions; i++)
        {
            printf("Data Description # %d (type=%d)\n", i, pDataDefs->arrDataDescriptions[i].type);
            if (pDataDefs->arrDataDescriptions[i].type == Descriptor_RigidBody)
            {
                // RigidBody
                sRigidBodyDescription* pRB = pDataDefs->arrDataDescriptions[i].Data.RigidBodyDescription;
                printf("RigidBody Name : %s\n", pRB->szName);
                printf("RigidBody ID : %d\n", pRB->ID);
                printf("RigidBody Parent ID : %d\n", pRB->parentID);
                printf("Parent Offset : %3.2f,%3.2f,%3.2f\n", pRB->offsetx, pRB->offsety, pRB->offsetz);

                if (pRB->MarkerPositions != NULL && pRB->MarkerRequiredLabels != NULL)
                {
                    for (int markerIdx = 0; markerIdx < pRB->nMarkers; ++markerIdx)
                    {
                        const MarkerData& markerPosition = pRB->MarkerPositions[markerIdx];
                        const int markerRequiredLabel = pRB->MarkerRequiredLabels[markerIdx];

                        printf("\tMarker #%d:\n", markerIdx);
                        printf("\t\tPosition: %.2f, %.2f, %.2f\n", markerPosition[0], markerPosition[1], markerPosition[2]);

                        if (markerRequiredLabel != 0)
                        {
                            printf("\t\tRequired active label: %d\n", markerRequiredLabel);
                        }
                    }
                }
            }
            else
            {
                printf("Unknown data type.\n");
                // Unknown
            }
        }
    }

    // Create data file for writing received stream into
    const char* szFile = "Client-output.pts";
    if (argc > 3)
        szFile = argv[3];

    g_outputFile = fopen(szFile, "w");
    if (!g_outputFile)
    {
        printf("Error opening output file %s.  Exiting.\n", szFile);
        exit(1);
    }

    if (pDataDefs)
    {
        _WriteHeader(g_outputFile, pDataDefs);
        NatNet_FreeDescriptions(pDataDefs);
        pDataDefs = NULL;
    }

    // Ready to receive marker stream!
    printf("\nClient is connected to server and listening for data...\n");
    bool bExit = false;
    while (const int c = _getch())
    {
        switch (c)
        {
        case 'q':
            bExit = true;
            break;
        case 'r':
            resetClient();
            break;
        case 'p':
            sServerDescription ServerDescription;
            memset(&ServerDescription, 0, sizeof(ServerDescription));
            g_pClient->GetServerDescription(&ServerDescription);
            if (!ServerDescription.HostPresent)
            {
                printf("Unable to connect to server. Host not present. Exiting.");
                return;
            }
            break;
        case 's':
        {
            printf("\n\n[SampleClient] Requesting Data Descriptions...");
            sDataDescriptions* pDataDefs = NULL;
            iResult = g_pClient->GetDataDescriptionList(&pDataDefs);
            if (iResult != ErrorCode_OK || pDataDefs == NULL)
            {
                printf("[SampleClient] Unable to retrieve Data Descriptions.");
            }
            else
            {
                printf("[SampleClient] Received %d Data Descriptions:\n", pDataDefs->nDataDescriptions);
            }
        }
        break;
        case 'm':	                        // change to multicast
            g_connectParams.connectionType = ConnectionType_Multicast;
            iResult = ConnectClient();
            if (iResult == ErrorCode_OK)
                printf("Client connection type changed to Multicast.\n\n");
            else
                printf("Error changing client connection type to Multicast.\n\n");
            break;
        case 'u':	                        // change to unicast
            g_connectParams.connectionType = ConnectionType_Unicast;
            iResult = ConnectClient();
            if (iResult == ErrorCode_OK)
                printf("Client connection type changed to Unicast.\n\n");
            else
                printf("Error changing client connection type to Unicast.\n\n");
            break;
        case 'c':                          // connect
            iResult = ConnectClient();
            break;
        case 'd':                          // disconnect
            // note: applies to unicast connections only - indicates to Motive to stop sending packets to that client endpoint
            iResult = g_pClient->SendMessageAndWait("Disconnect", &response, &nBytes);
            if (iResult == ErrorCode_OK)
                printf("[SampleClient] Disconnected");
            break;
        default:
            break;
        }
        if (bExit)
            break;
    }
    // Done - clean up.
    if (g_pClient)
    {
        g_pClient->Disconnect();
        delete g_pClient;
        g_pClient = NULL;
    }

    if (g_outputFile)
    {
        _WriteFooter(g_outputFile);
        fclose(g_outputFile);
        g_outputFile = NULL;
    }
    cameralock.unlock();
}

int main(int argc, char* argv[])
{
    *flag = 0;

    const char* serverAddress2 = "192.168.1.111"; // Horcy2 Robot's Ip address in the network
    const char* serverAddress3 = "192.168.1.146";//Horcy3
    const char* serverAddress4 = "192.168.1.145"; //Horcy4 Robot's Ip address in the network
    const char* serverAddress5 = "192.168.1.116"; //Horcy5
    const char* serverAddress6 = "192.168.1.132"; //Horcy6 Robot's Ip address in the network
    const char* serverAddress7 = "192.168.1.135";//Horcy7
    const char* serverAddress8 = "192.168.1.142";//Horcy8
    const char* serverAddress9 = "192.168.1.109";//Horcy9
    const char* serverAddress10 = "192.168.1.120";//Horcy10
    const char* serverAddress11 = "192.168.1.129";//Horcy11

    const int port2 = 50923;//horcy2
    const int port3 = 40923;//horcy3
    const int port4 = 60923;//horcy4
    const int port5 = 30923;//horcy5
    const int port6 = 6000;//horcy6
    const int port7 = 1000;//horcy7
    const int port8 = 2000;//horcy8
    const int port9 = 3000;//horcy9
    const int port10 = 5000;//horcy10
    const int port11 = 9923;//horcy11

    std::thread cameraThread(cameraTracking, argc, argv);
    Sleep(200);
    std::thread CollisionThread(cohesion_n_collision_control);
    std::thread thread1(robotControltest, serverAddress2, port2);
    std::thread thread2(robotControl2c, serverAddress3, port3);
    std::thread thread3(robotControl3c, serverAddress4, port4);
    std::thread thread4(robotControl4c, serverAddress5, port5);
    std::thread thread5(robotControl5c, serverAddress6, port6);
    std::thread thread6(robotControl6c, serverAddress7, port7);
    //std::thread thread7(robotControl7, serverAddress8, port8);
    //std::thread thread8(robotControl8, serverAddress9, port9);
    //std::thread thread9(robotControl9, serverAddress10, port10);
    //std::thread thread10(robotControl10, serverAddress11, port11);
    //std::thread Dog_thread(Dog_Tracking);
    // Wait for all threads to finish
    cameraThread.join();
    //Dog_thread.join();
    CollisionThread.join();
    thread1.join();
    thread2.join();
    thread3.join();
    thread4.join();
    thread5.join();
    thread6.join();
    //thread7.join();
    //thread8.join();
    //thread9.join();
    //thread10.join();
    //thread11.join();

    return 0;
}
