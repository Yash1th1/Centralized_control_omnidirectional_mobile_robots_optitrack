#include "utils.h"

void printRigidBodies(const sFrameOfMocapData* data)
{
    // Rigid Bodies
    std::cout << std::format("Rigid Bodies [Count={}]\n", data->nRigidBodies);
    for (int i = 0; i < data->nRigidBodies; i++)
    {
        // params
        // 0x01 : bool, rigid body was successfully tracked in this frame
        bool bTrackingValid = data->RigidBodies[i].params & 0x01;

        printf("Rigid Body [ID=%d  Error=%3.2f  Valid=%d]\n", data->RigidBodies[i].ID, data->RigidBodies[i].MeanError, bTrackingValid);
        std::cout << std::format("  Position(x={:.2f}, y={:.2f}, z={:.2f})\n", data->RigidBodies[i].x,
            data->RigidBodies[i].y, data->RigidBodies[i].z);
        std::cout << std::format("  Orientation(qx={:.2f}, qy={:.2f}, qz={:.2f}, qw={:.2f})\n",
            data->RigidBodies[i].qx, data->RigidBodies[i].qy, data->RigidBodies[i].qz, data->RigidBodies[i].qw);
        std::cout << "--------------------------------------------------------------" << std::endl;
    }

    order = EulOrdXYZr;
    ea = Eul_FromQuat(q, order);

    ea.x = NATUtils::RadiansToDegrees(ea.x);
    ea.y = NATUtils::RadiansToDegrees(ea.y);
    ea.z = NATUtils::RadiansToDegrees(ea.z);
}