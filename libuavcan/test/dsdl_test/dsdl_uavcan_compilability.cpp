/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <gtest/gtest.h>

#include <uavcan/Timestamp.hpp>
#include <uavcan/FigureOfMerit.hpp>
#include <uavcan/mavlink/Message.hpp>
#include <uavcan/protocol/ComputeAggregateTypeSignature.hpp>
#include <uavcan/protocol/GetProtocolStatistics.hpp>
#include <uavcan/protocol/Panic.hpp>
#include <uavcan/protocol/RestartNode.hpp>
#include <uavcan/protocol/GlobalTimeSync.hpp>
#include <uavcan/protocol/DataTypeKind.hpp>
#include <uavcan/protocol/GlobalDiscoveryRequest.hpp>
#include <uavcan/protocol/GetDataTypeInfo.hpp>
#include <uavcan/protocol/Version.hpp>
#include <uavcan/protocol/NodeStatus.hpp>
#include <uavcan/protocol/GetNodeInfo.hpp>
#include <uavcan/protocol/debug/LogMessage.hpp>
#include <uavcan/protocol/debug/StartHilSimulation.hpp>
#include <uavcan/protocol/debug/KeyValue.hpp>
#include <uavcan/protocol/file/Path.hpp>
#include <uavcan/protocol/file/Read.hpp>
#include <uavcan/protocol/file/Delete.hpp>
#include <uavcan/protocol/file/Errno.hpp>
#include <uavcan/protocol/file/BeginFirmwareUpdate.hpp>
#include <uavcan/protocol/file/List.hpp>
#include <uavcan/protocol/file/BeginTransfer.hpp>
#include <uavcan/protocol/file/Crc.hpp>
#include <uavcan/protocol/file/GetInfo.hpp>

#include <root_ns_a/Deep.hpp>


TEST(Dsdl, Streaming)
{
    std::ostringstream os;

    uavcan::mavlink::Message mavlink;
    os << mavlink << std::endl << "==========" << std::endl;

    mavlink.compid = 12;
    mavlink.seq = 42;
    mavlink.payload = "Here goes payload";
    os << mavlink << std::endl << "==========" << std::endl;

    uavcan::protocol::GetNodeInfo::Response get_node_info_rsp;
    os << get_node_info_rsp << std::endl << "==========" << std::endl;

    root_ns_a::Deep ps;
    ps.a.resize(2);
    os << ps << std::endl << "==========" << std::endl;

    static const std::string Reference = "seq: 0\n"
        "sysid: 0\n"
        "compid: 0\n"
        "msgid: 0\n"
        "payload: \"\"\n"
        "==========\n"
        "seq: 42\n"
        "sysid: 0\n"
        "compid: 12\n"
        "msgid: 0\n"
        "payload: \"Here goes payload\"\n"
        "==========\n"
        "software_version: \n"
        "  major: 0\n"
        "  minor: 0\n"
        "hardware_version: \n"
        "  major: 0\n"
        "  minor: 0\n"
        "uavcan_version: \n"
        "  major: 0\n"
        "  minor: 0\n"
        "status: \n"
        "  uptime_sec: 0\n"
        "  status_code: 0\n"
        "name: \"\"\n"
        "==========\n"
        "c: 0\n"
        "str: \"\"\n"
        "a: \n"
        "  - \n"
        "    scalar: 0\n"
        "    vector: \n"
        "      - \n"
        "        vector: [0, 0, 0, 0]\n"
        "        bools: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]\n"
        "      - \n"
        "        vector: [0, 0, 0, 0]\n"
        "        bools: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]\n"
        "      - \n"
        "        vector: [0, 0, 0, 0]\n"
        "        bools: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]\n"
        "  - \n"
        "    scalar: 0\n"
        "    vector: \n"
        "      - \n"
        "        vector: [0, 0, 0, 0]\n"
        "        bools: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]\n"
        "      - \n"
        "        vector: [0, 0, 0, 0]\n"
        "        bools: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]\n"
        "      - \n"
        "        vector: [0, 0, 0, 0]\n"
        "        bools: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]\n"
        "b: \n"
        "  - \n"
        "    vector: [0, 0, 0, 0]\n"
        "    bools: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]\n"
        "  - \n"
        "    vector: [0, 0, 0, 0]\n"
        "    bools: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]\n"
        "  - \n"
        "    vector: [0, 0, 0, 0]\n"
        "    bools: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]\n"
        "==========\n";
    ASSERT_EQ(Reference, os.str());
}
