# Bluetooth Classic SPP On Pico 2 W: Practical Notes

## Why This Note Exists

This project spent a full bring-up cycle proving Bluetooth Classic SPP on:

- Raspberry Pi Pico 2 W
- CYW43439
- Pico SDK 2.2.0
- BTstack
- RP2350 ARM build path

The goal of this note is to capture the parts that are useful beyond this project so the next person does not have to rediscover them the hard way.

## Short Version

Yes, Bluetooth Classic SPP can work on Pico 2 W on this stack.

But one easy-to-miss BTstack config detail can make the symptoms very misleading:

- `MAX_NR_SERVICE_RECORD_ITEMS`

If this service-record item pool is missing or too small, you can see all of these at once:

- Classic radio up
- pairing works
- SDP record bytes look valid locally
- `sdp_register_service(...)` appears fine
- client reaches SDP
- client never opens RFCOMM

That was the main trap.

## Platform Context

Working context in this project:

- Pico SDK: `2.2.0`
- Board: `pico2_w`
- Platform: `rp2350-arm-s`
- Controller: `CYW43439`
- Stack: BTstack via Pico SDK

Important rule discovered during bring-up:

- on this CYW43-based path, Classic-enabled builds still need BLE core support present in BTstack config/linkage

So a practical Classic-capable config was not a pure "Classic only" minimal build.

## The Misleading Failure Pattern

Before the real fix, the log looked like this:

1. discoverable
2. pairable
3. pairing succeeds
4. client opens `L2CAP PSM 0x0001` (SDP)
5. client never proceeds to `PSM 0x0003` / RFCOMM
6. disconnect follows

This can make you suspect:

- SSP IO capability
- link key handling
- `sm_init()`
- malformed SDP record
- Android cache weirdness
- Windows client weirdness

Those were all plausible, but not the root cause.

## The Real Root Cause

The missing piece was BTstack SDP server record-item storage.

The key config addition was:

```c
#define MAX_NR_SERVICE_RECORD_ITEMS 4
```

In this project that lives in:

- `include/btstack_config.h`

Without it, the project still had:

- a generated SPP record
- valid-looking SDP bytes
- an apparently successful `sdp_register_service(...)`

But the live SDP server path did not have the service-record item capacity needed for clients to discover and use the service correctly.

## Minimal Practical BTstack Config Pieces

The relevant working Classic-capable config shape looked like this:

```c
#define ENABLE_CLASSIC 1
#define ENABLE_RFCOMM 1
#define ENABLE_SDP_SERVER 1
#define MAX_NR_SDP_RECORDS 4
#define MAX_NR_SERVICE_RECORD_ITEMS 4
#define MAX_NR_HCI_CONNECTIONS 2
#define MAX_NR_L2CAP_CHANNELS 6
#define MAX_NR_L2CAP_SERVICES 3
```

And on this Pico/CYW43 path, BLE support was still kept enabled too.

## Practical SPP Setup That Worked

The working SPP setup pattern was standard BTstack:

```c
l2cap_init();
sm_init();
rfcomm_init();
sdp_init();
rfcomm_register_service(packet_handler, kRfcommServerChannel, 0xffff);

memset(g_spp_service_buffer, 0, sizeof(g_spp_service_buffer));
spp_create_sdp_record(
    g_spp_service_buffer,
    sdp_create_service_record_handle(),
    kRfcommServerChannel,
    "IDRIVE Classic Smoke");

btstack_assert(de_get_len(g_spp_service_buffer) <= sizeof(g_spp_service_buffer));
sdp_register_service(g_spp_service_buffer);
```

The important point is that this alone may still not work if the BTstack config pool sizing is incomplete.

## Headless Device Settings That Were Reasonable

These settings were fine and worth keeping for a headless smoke target:

```c
gap_set_class_of_device(0x001F00);
gap_ssp_set_io_capability(SSP_IO_CAPABILITY_NO_INPUT_NO_OUTPUT);
gap_discoverable_control(1);
gap_connectable_control(1);
```

These were good cleanup changes, but they were not the decisive fix.

## How To Diagnose The Problem Properly

The most useful sequence was:

1. enable USB/HCI dump logging
2. confirm whether the client reaches SDP (`PSM 0x0001`)
3. confirm whether RFCOMM (`PSM 0x0003`) is ever attempted
4. inspect SDP server matching/response behavior

The key observation was:

- the client did reach SDP
- but RFCOMM was never attempted

That narrowed the issue away from generic pairing or radio bring-up.

## What Finally Proved It

After adding the missing service-record pool budget, the observed behavior changed to:

1. SDP search matched `0x1101`
2. full SPP record was returned
3. RFCOMM connection opened
4. Classic RX/TX worked

In this project, dual smoke then also worked with:

- BLE active
- Classic active
- both simultaneously

## Practical Advice For Other Projects

If Classic SPP on Pico 2 W looks half-alive, check these first:

1. BTstack pool sizing, especially:
   - `MAX_NR_SERVICE_RECORD_ITEMS`
2. whether the client reaches SDP at all
3. whether the client ever attempts RFCOMM
4. whether you accidentally depend on hidden out-of-repo SDK patches during debugging

And do not stop at "the SDP record bytes look fine locally". That alone is not enough.

## Firmware Paths From This Project

Relevant project files:

- `include/btstack_config.h`
- `src/smoke/btstack_classic_smoke.cpp`
- `src/smoke/btstack_dual_smoke.cpp`
- `src/transport/transport_backend_btstack.cpp`

## Closing Thought

The biggest lesson here is simple:

- valid local SDP record bytes do not prove the live BTstack SDP server can actually serve that record to the client correctly

On this project, the missing record-item pool was the difference between:

- pair but never open RFCOMM

and:

- real working Bluetooth Classic SPP.
