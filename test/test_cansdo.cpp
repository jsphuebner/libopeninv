/*
 * This file is part of the libopeninv project.
 *
 * Copyright (C) 2025 Johannes Huebner <dev@johanneshuebner.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
// Pull in system stdio *first* so printf/sprintf get C linkage.  The libopeninv
// printf.h re-declares them without extern "C", which conflicts on modern hosts.
// Declaring IPutChar ourselves and setting the include guard prevents printf.h
// from being processed a second time, avoiding the linkage mismatch.
#include <cstdio>
class IPutChar { public: virtual void PutChar(char c) = 0; };
#define PRINTF_H_INCLUDED

#include "cansdo.h"
#include "canmap.h"
#include "params.h"
#include "my_fp.h"
#include "stub_canhardware.h"
#include "test.h"

#include <memory>
#include <cstdint>

class CanSdoTest : public UnitTest
{
public:
    explicit CanSdoTest(const std::list<VoidFunction>* cases) : UnitTest(cases) {}
    virtual void TestCaseSetup();
};

static std::unique_ptr<CanStub> canStub;
static std::unique_ptr<CanMap>  canMap;
static std::unique_ptr<CanSdo>  canSdo;

void CanSdoTest::TestCaseSetup()
{
    canStub = std::make_unique<CanStub>();
    canMap  = std::make_unique<CanMap>(canStub.get(), false);
    canSdo  = std::make_unique<CanSdo>(canStub.get(), canMap.get());
    Param::LoadDefaults();
}

// Default node ID is 1, so SDO requests arrive on CAN ID 0x601
static const uint32_t SdoReqId  = 0x601;
static const uint32_t SdoRepId  = 0x581;

// Build and send an SDO request to the CanSdo instance under test
static void SendSdoRequest(uint8_t cmd, uint16_t index, uint8_t subIndex, uint32_t data)
{
    uint32_t frame[2];
    CanSdo::SdoFrame* sdo = (CanSdo::SdoFrame*)frame;
    sdo->cmd      = cmd;
    sdo->index    = index;
    sdo->subIndex = subIndex;
    sdo->data     = data;
    canStub->HandleRx(SdoReqId, frame, 8);
}

// Access the last CAN frame sent by CanSdo as an SdoFrame
static CanSdo::SdoFrame* GetReply()
{
    return (CanSdo::SdoFrame*)&canStub->m_data[0];
}

// ---------------------------------------------------------------------------
// Parameter read/write via SDO index 0x2000
// ---------------------------------------------------------------------------

static void sdo_read_param()
{
    Param::SetFloat(Param::ocurlim, 42.0f);
    SendSdoRequest(SDO_READ, 0x2000, Param::ocurlim, 0);

    ASSERT(canStub->m_canId == SdoRepId);
    ASSERT(GetReply()->cmd == SDO_READ_REPLY);
    ASSERT(GetReply()->data == (uint32_t)Param::Get(Param::ocurlim));
}

static void sdo_write_param()
{
    s32fp newVal = FP_FROMFLT(50.0f);
    SendSdoRequest(SDO_WRITE, 0x2000, Param::ocurlim, (uint32_t)newVal);

    ASSERT(canStub->m_canId == SdoRepId);
    ASSERT(GetReply()->cmd == SDO_WRITE_REPLY);
    ASSERT(Param::Get(Param::ocurlim) == newVal);
}

static void sdo_write_param_out_of_range()
{
    // ocurlim max is 65536; use a value clearly above it
    s32fp outOfRange = FP_FROMFLT(100000.0f);
    SendSdoRequest(SDO_WRITE, 0x2000, Param::ocurlim, (uint32_t)outOfRange);

    ASSERT(canStub->m_canId == SdoRepId);
    ASSERT(GetReply()->cmd == SDO_ABORT);
    ASSERT(GetReply()->data == SDO_ERR_RANGE);
}

static void sdo_read_invalid_param_index()
{
    SendSdoRequest(SDO_READ, 0x2000, Param::PARAM_LAST, 0);

    ASSERT(canStub->m_canId == SdoRepId);
    ASSERT(GetReply()->cmd == SDO_ABORT);
    ASSERT(GetReply()->data == SDO_ERR_INVIDX);
}

static void sdo_write_invalid_param_index()
{
    SendSdoRequest(SDO_WRITE, 0x2000, Param::PARAM_LAST, FP_FROMFLT(1.0f));

    ASSERT(canStub->m_canId == SdoRepId);
    ASSERT(GetReply()->cmd == SDO_ABORT);
    ASSERT(GetReply()->data == SDO_ERR_INVIDX);
}

// ---------------------------------------------------------------------------
// Parameter access by unique ID via SDO index 0x21xx
// ---------------------------------------------------------------------------

static void sdo_read_param_by_uid()
{
    // ocurlim has UID = 22 (defined in param_prj.h)
    const uint16_t ocurlimUid = Param::GetAttrib(Param::ocurlim)->id;
    Param::SetFloat(Param::ocurlim, 77.0f);

    // Index encodes high byte of UID; subIndex = low byte of UID
    uint16_t sdoIndex   = 0x2100 | ((ocurlimUid >> 8) & 0xFF);
    uint8_t  sdoSubIdx  = ocurlimUid & 0xFF;

    SendSdoRequest(SDO_READ, sdoIndex, sdoSubIdx, 0);

    ASSERT(canStub->m_canId == SdoRepId);
    ASSERT(GetReply()->cmd == SDO_READ_REPLY);
    ASSERT(GetReply()->data == (uint32_t)Param::Get(Param::ocurlim));
}

static void sdo_write_param_by_uid()
{
    const uint16_t ocurlimUid = Param::GetAttrib(Param::ocurlim)->id;
    s32fp newVal = FP_FROMFLT(33.0f);

    uint16_t sdoIndex  = 0x2100 | ((ocurlimUid >> 8) & 0xFF);
    uint8_t  sdoSubIdx = ocurlimUid & 0xFF;

    SendSdoRequest(SDO_WRITE, sdoIndex, sdoSubIdx, (uint32_t)newVal);

    ASSERT(canStub->m_canId == SdoRepId);
    ASSERT(GetReply()->cmd == SDO_WRITE_REPLY);
    ASSERT(Param::Get(Param::ocurlim) == newVal);
}

static void sdo_read_unknown_uid()
{
    // UID 0xFFFF should not exist
    SendSdoRequest(SDO_READ, 0x21FF, 0xFF, 0);

    ASSERT(canStub->m_canId == SdoRepId);
    ASSERT(GetReply()->cmd == SDO_ABORT);
    ASSERT(GetReply()->data == SDO_ERR_INVIDX);
}

// ---------------------------------------------------------------------------
// CAN map add via SDO (TX: 0x3000, RX: 0x3001)
// ---------------------------------------------------------------------------

// Encode an SDO map add step-1 data word from UID, bit offset, and bit length
static uint32_t MakeMapStep1(uint16_t uid, uint8_t offsetBits, int8_t numBits)
{
    return (uint32_t)uid | ((uint32_t)offsetBits << 16) | ((uint32_t)(uint8_t)numBits << 24);
}

// Encode an SDO map add step-2 data word from gain (x1000) and offset byte
static uint32_t MakeMapStep2(int32_t gainFixed, int8_t offset)
{
    return (uint32_t)(gainFixed & 0xFFFFFF) | ((uint32_t)(uint8_t)offset << 24);
}

static void sdo_add_tx_can_map()
{
    const uint32_t cobId = 0x200;
    const uint16_t uid   = Param::GetAttrib(Param::ocurlim)->id; // 22

    // Step 0: set COB-ID
    SendSdoRequest(SDO_WRITE, 0x3000, 0, cobId);
    ASSERT(GetReply()->cmd == SDO_WRITE_REPLY);

    // Step 1: param UID, bit offset, bit length
    SendSdoRequest(SDO_WRITE, 0x3000, 1, MakeMapStep1(uid, 0, 8));
    ASSERT(GetReply()->cmd == SDO_WRITE_REPLY);

    // Step 2: gain (1.0 → 1000) and offset
    SendSdoRequest(SDO_WRITE, 0x3000, 2, MakeMapStep2(1000, 0));
    ASSERT(GetReply()->cmd == SDO_WRITE_REPLY);

    // Verify the mapping was actually added by sending the mapped frame
    Param::SetFloat(Param::ocurlim, 42.0f);
    canMap->SendAll();
    ASSERT(canStub->m_canId == cobId);
    ASSERT(canStub->m_data[0] == 42);
}

static void sdo_add_rx_can_map()
{
    const uint32_t cobId = 0x300;
    const uint16_t uid   = Param::GetAttrib(Param::ocurlim)->id;

    SendSdoRequest(SDO_WRITE, 0x3001, 0, cobId);
    ASSERT(GetReply()->cmd == SDO_WRITE_REPLY);

    SendSdoRequest(SDO_WRITE, 0x3001, 1, MakeMapStep1(uid, 0, 8));
    ASSERT(GetReply()->cmd == SDO_WRITE_REPLY);

    SendSdoRequest(SDO_WRITE, 0x3001, 2, MakeMapStep2(1000, 0));
    ASSERT(GetReply()->cmd == SDO_WRITE_REPLY);

    // Verify the mapping was stored in CanMap using FindMap
    uint32_t foundCanId = 0;
    uint8_t  start = 0;
    int8_t   length = 0;
    float    gain = 0.0f;
    int8_t   offset = 0;
    bool     rx = false;
    ASSERT(canMap->FindMap(Param::ocurlim, foundCanId, start, length, gain, offset, rx));
    ASSERT(foundCanId == cobId);
    ASSERT(rx == true);
}

static void sdo_add_tx_can_map_invalid_cobid()
{
    // cobId 0x40000000 exceeds the allowed range on both the
    // < 0x20000000 and the force-extended path → the reply is SDO_ABORT
    SendSdoRequest(SDO_WRITE, 0x3000, 0, 0x40000000U);
    ASSERT(GetReply()->cmd == SDO_ABORT);

    // mapId is 0xFFFFFFFF after the failed step 0, so step 1 also aborts
    const uint16_t uid = Param::GetAttrib(Param::ocurlim)->id;
    SendSdoRequest(SDO_WRITE, 0x3000, 1, MakeMapStep1(uid, 0, 8));
    ASSERT(GetReply()->cmd == SDO_ABORT);
}

static void sdo_add_tx_can_map_unknown_uid()
{
    SendSdoRequest(SDO_WRITE, 0x3000, 0, 0x200);
    ASSERT(GetReply()->cmd == SDO_WRITE_REPLY);

    // UID 0xFFFF doesn't correspond to any parameter
    SendSdoRequest(SDO_WRITE, 0x3000, 1, MakeMapStep1(0xFFFF, 0, 8));
    ASSERT(GetReply()->cmd == SDO_ABORT);
    ASSERT(GetReply()->data == SDO_ERR_INVIDX);
}

// ---------------------------------------------------------------------------
// CAN map read/delete via SDO index 0x31xx
// ---------------------------------------------------------------------------

static void sdo_read_tx_can_map_cobid()
{
    const uint32_t cobId = 0x123;
    canMap->AddSend(Param::ocurlim, cobId, 0, 8, 1.0f, 0);

    // subIndex 0 returns the COB-ID for the first TX message (ididx=0)
    SendSdoRequest(SDO_READ, 0x3100, 0, 0);

    ASSERT(canStub->m_canId == SdoRepId);
    ASSERT(GetReply()->cmd == SDO_READ_REPLY);
    ASSERT(GetReply()->data == cobId);
}

static void sdo_read_tx_can_map_item()
{
    const uint32_t cobId = 0x123;
    canMap->AddSend(Param::ocurlim, cobId, 8, 16, 2.0f, 3);

    // subIndex 1 (odd) returns param UID, offsetBits, numBits for itemIdx 0
    SendSdoRequest(SDO_READ, 0x3100, 1, 0);
    ASSERT(GetReply()->cmd == SDO_READ_REPLY);
    uint16_t expectedUid = Param::GetAttrib(Param::ocurlim)->id;
    uint32_t expectedData = expectedUid | (8U << 16) | ((uint32_t)(uint8_t)16 << 24);
    ASSERT(GetReply()->data == expectedData);

    // subIndex 2 (even) returns gain (x1000) and offset
    SendSdoRequest(SDO_READ, 0x3100, 2, 0);
    ASSERT(GetReply()->cmd == SDO_READ_REPLY);
    int32_t gainFixed = (int32_t)(2.0f * 1000.0f);
    uint32_t expectedGainOffset = (uint32_t)(gainFixed & 0xFFFFFF) | ((uint32_t)(uint8_t)3 << 24);
    ASSERT(GetReply()->data == expectedGainOffset);
}

static void sdo_read_tx_can_map_out_of_range()
{
    // No mapping added; reading from empty map should abort
    SendSdoRequest(SDO_READ, 0x3100, 0, 0);

    ASSERT(canStub->m_canId == SdoRepId);
    ASSERT(GetReply()->cmd == SDO_ABORT);
    ASSERT(GetReply()->data == SDO_ERR_INVIDX);
}

static void sdo_delete_tx_can_map()
{
    canMap->AddSend(Param::ocurlim, 0x123, 0, 8, 1.0f, 0);

    // Writing 0 to subIndex 1 of an existing entry deletes it
    SendSdoRequest(SDO_WRITE, 0x3100, 1, 0);

    ASSERT(canStub->m_canId == SdoRepId);
    ASSERT(GetReply()->cmd == SDO_WRITE_REPLY);

    // Confirm deletion: reading the entry should now abort
    SendSdoRequest(SDO_READ, 0x3100, 0, 0);
    ASSERT(GetReply()->cmd == SDO_ABORT);
}

static void sdo_read_rx_can_map_cobid()
{
    const uint32_t cobId = 0x321;
    canMap->AddRecv(Param::ocurlim, cobId, 0, 8, 1.0f, 0);

    // RX maps use bit 7 of the index: 0x3180
    SendSdoRequest(SDO_READ, 0x3180, 0, 0);

    ASSERT(canStub->m_canId == SdoRepId);
    ASSERT(GetReply()->cmd == SDO_READ_REPLY);
    ASSERT(GetReply()->data == cobId);
}

// ---------------------------------------------------------------------------
// Error message SDO (index 0x5003 / 0x5004)
// ---------------------------------------------------------------------------

static void sdo_read_error_num()
{
    SendSdoRequest(SDO_READ, 0x5003, 0, 0);

    ASSERT(canStub->m_canId == SdoRepId);
    ASSERT(GetReply()->cmd == SDO_READ_REPLY);
}

static void sdo_read_error_time()
{
    SendSdoRequest(SDO_READ, 0x5004, 0, 0);

    ASSERT(canStub->m_canId == SdoRepId);
    ASSERT(GetReply()->cmd == SDO_READ_REPLY);
}

static void sdo_write_error_num_aborts()
{
    SendSdoRequest(SDO_WRITE, 0x5003, 0, 0);

    ASSERT(canStub->m_canId == SdoRepId);
    ASSERT(GetReply()->cmd == SDO_ABORT);
    ASSERT(GetReply()->data == SDO_ERR_INVIDX);
}

static void sdo_write_error_time_aborts()
{
    SendSdoRequest(SDO_WRITE, 0x5004, 0, 0);

    ASSERT(canStub->m_canId == SdoRepId);
    ASSERT(GetReply()->cmd == SDO_ABORT);
    ASSERT(GetReply()->data == SDO_ERR_INVIDX);
}

// ---------------------------------------------------------------------------
// Unknown SDO index goes to user space
// ---------------------------------------------------------------------------

static void sdo_unknown_index_goes_to_user_space()
{
    // Index 0x4000 is not handled by CanSdo itself
    SendSdoRequest(SDO_WRITE, 0x4000, 0, 0xDEADBEEF);

    // No CAN reply should be sent; the pending user-space SDO should be set
    CanSdo::SdoFrame* pending = canSdo->GetPendingUserspaceSdo();
    ASSERT(pending != nullptr);
    ASSERT(pending->index == 0x4000);
    ASSERT(pending->data == 0xDEADBEEF);
}

static void sdo_reply_sent_via_send_sdo_reply()
{
    SendSdoRequest(SDO_WRITE, 0x4000, 0, 0x1234);

    CanSdo::SdoFrame* pending = canSdo->GetPendingUserspaceSdo();
    ASSERT(pending != nullptr);

    // User space fills in the reply and calls SendSdoReply
    pending->cmd  = SDO_WRITE_REPLY;
    pending->data = 0;
    canSdo->SendSdoReply(pending);

    // After sending the reply, the pending flag must be cleared
    ASSERT(canSdo->GetPendingUserspaceSdo() == nullptr);
    ASSERT(canStub->m_canId == SdoRepId);
    ASSERT(GetReply()->cmd == SDO_WRITE_REPLY);
}

// ---------------------------------------------------------------------------
// Node ID change
// ---------------------------------------------------------------------------

static void sdo_request_ignored_for_wrong_node_id()
{
    // With node ID 1, requests for node 2 (0x602) should be silently ignored
    s32fp before = Param::Get(Param::ocurlim);

    uint32_t frame[2];
    CanSdo::SdoFrame* sdo = (CanSdo::SdoFrame*)frame;
    sdo->cmd      = SDO_WRITE;
    sdo->index    = 0x2000;
    sdo->subIndex = Param::ocurlim;
    sdo->data     = (uint32_t)FP_FROMFLT(99.0f);
    canStub->HandleRx(0x602, frame, 8); // Wrong node ID

    ASSERT(Param::Get(Param::ocurlim) == before);
}

static void sdo_request_processed_after_set_node_id()
{
    canSdo->SetNodeId(2);

    s32fp newVal = FP_FROMFLT(55.0f);
    uint32_t frame[2];
    CanSdo::SdoFrame* sdo = (CanSdo::SdoFrame*)frame;
    sdo->cmd      = SDO_WRITE;
    sdo->index    = 0x2000;
    sdo->subIndex = Param::ocurlim;
    sdo->data     = (uint32_t)newVal;
    // After SetNodeId(2), requests for node 2 (0x602) must be handled
    canStub->HandleRx(0x602, frame, 8);

    ASSERT(Param::Get(Param::ocurlim) == newVal);
}

// ---------------------------------------------------------------------------
// String / print upload initiation (SDO_INDEX_STRINGS = 0x5001)
// ---------------------------------------------------------------------------

static void sdo_read_strings_initiates_print_request()
{
    // Before reading, no print request is pending
    ASSERT(canSdo->GetPrintRequest() == -1);

    SendSdoRequest(SDO_READ, 0x5001, 3, 0);

    ASSERT(canStub->m_canId == SdoRepId);
    // The reply should be an upload initiation (SDO_RESPONSE_UPLOAD | SDO_SIZE_SPECIFIED)
    ASSERT(GetReply()->cmd == (SDO_RESPONSE_UPLOAD | SDO_SIZE_SPECIFIED));
    // GetPrintRequest() should now return the requested sub-index
    ASSERT(canSdo->GetPrintRequest() == 3);
}

// ---------------------------------------------------------------------------
// Parameter flags via SDO index SDO_INDEX_PARAM_FLAGS (0x2200)
// ---------------------------------------------------------------------------

static void sdo_read_param_flags_default()
{
    // By default flags are FLAG_NONE (0)
    SendSdoRequest(SDO_READ, 0x2200, Param::ocurlim, 0);

    ASSERT(canStub->m_canId == SdoRepId);
    ASSERT(GetReply()->cmd == SDO_READ_REPLY);
    ASSERT(GetReply()->data == (uint32_t)Param::FLAG_NONE);
}

static void sdo_write_and_read_param_flags()
{
    // Set FLAG_HIDDEN on ocurlim
    SendSdoRequest(SDO_WRITE, 0x2200, Param::ocurlim, (uint32_t)Param::FLAG_HIDDEN);

    ASSERT(canStub->m_canId == SdoRepId);
    ASSERT(GetReply()->cmd == SDO_WRITE_REPLY);
    ASSERT(Param::GetFlag(Param::ocurlim) == Param::FLAG_HIDDEN);

    // Read it back
    SendSdoRequest(SDO_READ, 0x2200, Param::ocurlim, 0);
    ASSERT(GetReply()->cmd == SDO_READ_REPLY);
    ASSERT(GetReply()->data == (uint32_t)Param::FLAG_HIDDEN);
}

static void sdo_write_param_flags_clear()
{
    // Pre-set the flag, then clear it via SDO
    Param::SetFlag(Param::ocurlim, Param::FLAG_HIDDEN);
    SendSdoRequest(SDO_WRITE, 0x2200, Param::ocurlim, (uint32_t)Param::FLAG_NONE);

    ASSERT(canStub->m_canId == SdoRepId);
    ASSERT(GetReply()->cmd == SDO_WRITE_REPLY);
    ASSERT(Param::GetFlag(Param::ocurlim) == Param::FLAG_NONE);
}

static void sdo_read_param_flags_invalid_index()
{
    SendSdoRequest(SDO_READ, 0x2200, Param::PARAM_LAST, 0);

    ASSERT(canStub->m_canId == SdoRepId);
    ASSERT(GetReply()->cmd == SDO_ABORT);
    ASSERT(GetReply()->data == SDO_ERR_INVIDX);
}

static void sdo_write_param_flags_invalid_index()
{
    SendSdoRequest(SDO_WRITE, 0x2200, Param::PARAM_LAST, (uint32_t)Param::FLAG_HIDDEN);

    ASSERT(canStub->m_canId == SdoRepId);
    ASSERT(GetReply()->cmd == SDO_ABORT);
    ASSERT(GetReply()->data == SDO_ERR_INVIDX);
}

// ---------------------------------------------------------------------------
// Test registration
// ---------------------------------------------------------------------------

REGISTER_TEST(
    CanSdoTest,
    sdo_read_param,
    sdo_write_param,
    sdo_write_param_out_of_range,
    sdo_read_invalid_param_index,
    sdo_write_invalid_param_index,
    sdo_read_param_by_uid,
    sdo_write_param_by_uid,
    sdo_read_unknown_uid,
    sdo_add_tx_can_map,
    sdo_add_rx_can_map,
    sdo_add_tx_can_map_invalid_cobid,
    sdo_add_tx_can_map_unknown_uid,
    sdo_read_tx_can_map_cobid,
    sdo_read_tx_can_map_item,
    sdo_read_tx_can_map_out_of_range,
    sdo_delete_tx_can_map,
    sdo_read_rx_can_map_cobid,
    sdo_read_error_num,
    sdo_read_error_time,
    sdo_write_error_num_aborts,
    sdo_write_error_time_aborts,
    sdo_unknown_index_goes_to_user_space,
    sdo_reply_sent_via_send_sdo_reply,
    sdo_request_ignored_for_wrong_node_id,
    sdo_request_processed_after_set_node_id,
    sdo_read_strings_initiates_print_request,
    sdo_read_param_flags_default,
    sdo_write_and_read_param_flags,
    sdo_write_param_flags_clear,
    sdo_read_param_flags_invalid_index,
    sdo_write_param_flags_invalid_index
);
