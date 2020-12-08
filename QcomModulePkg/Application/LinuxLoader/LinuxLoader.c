/*
 * Copyright (c) 2009, Google Inc.
 * All rights reserved.
 *
 * Copyright (c) 2009-2018, The Linux Foundation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of The Linux Foundation nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NON-INFRINGEMENT ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "AutoGen.h"
#include "BootLinux.h"
#include "BootStats.h"
#include "KeyPad.h"
#include "LinuxLoaderLib.h"
#include <FastbootLib/FastbootMain.h>
#include <Library/DeviceInfo.h>
#include <Library/DrawUI.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/PartitionTableUpdate.h>
#include <Library/ShutdownServices.h>
#include <Library/StackCanary.h>

#define Dualboot

#ifdef Dualboot
#include <../../../AbmPkg/Include/lvgl.h>
#endif

#define MAX_APP_STR_LEN 64
#define MAX_NUM_FS 10
#define DEFAULT_STACK_CHK_GUARD 0xc0c0c0c0

STATIC BOOLEAN BootReasonAlarm = FALSE;
STATIC BOOLEAN BootIntoFastboot = FALSE;
STATIC BOOLEAN BootIntoRecovery = FALSE;

STATIC VOID* UnSafeStackPtr;

STATIC EFI_STATUS __attribute__ ( (no_sanitize ("safe-stack")))
AllocateUnSafeStackPtr (VOID)
{

  EFI_STATUS Status = EFI_SUCCESS;

  UnSafeStackPtr = AllocatePool (BOOT_LOADER_MAX_UNSAFE_STACK_SIZE);
  if (UnSafeStackPtr == NULL) {
    DEBUG ((EFI_D_ERROR, "Failed to Allocate memory for UnSafeStack \n"));
    Status = EFI_OUT_OF_RESOURCES;
    return Status;
  }

  UnSafeStackPtr += BOOT_LOADER_MAX_UNSAFE_STACK_SIZE;

  return Status;
}

EFI_SYSTEM_TABLE                    *gST1;
EFI_BOOT_SERVICES             *gBS1;
EFI_GRAPHICS_OUTPUT_PROTOCOL *mGop;
lv_disp_drv_t                 mDispDrv;
lv_indev_drv_t                mFakeInputDrv;
lv_disp_t * disp;

static void EfiGopBltFlush(
    lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p)
{
  mGop->Blt(
      mGop, (EFI_GRAPHICS_OUTPUT_BLT_PIXEL *)color_p, EfiBltBufferToVideo, 0, 0,
      area->x1, area->y1, area->x2 - area->x1 + 1, area->y2 - area->y1 + 1, 0);

  lv_disp_flush_ready(disp_drv);
}

//Read keys state
bool key_read(lv_indev_drv_t * drv, lv_indev_data_t*data)
{
  EFI_INPUT_KEY  Key;

  //Read keys
  gST1->ConIn->ReadKeyStroke (gST1->ConIn, &Key);

  //Vol up
  data->key = LV_KEY_UP;
  if (Key.ScanCode==SCAN_UP){
      data->state = LV_INDEV_STATE_PR;
      return false;
  } 

  //Vol down 
  data->key = LV_KEY_DOWN;
  if (Key.ScanCode==SCAN_DOWN){
      data->state = LV_INDEV_STATE_PR;
      return false;
  } 

  //Pwr key
  data->key = LV_KEY_ENTER;
  if (Key.ScanCode==SCAN_SUSPEND){
      data->state = LV_INDEV_STATE_PR;
      return false;
  } 
}

//This function is to return the Unsafestack ptr address
VOID** __attribute__ ( (no_sanitize ("safe-stack")))
__safestack_pointer_address (VOID)
{

  return (VOID**) &UnSafeStackPtr;
}

// This function is used to Deactivate MDTP by entering recovery UI

STATIC EFI_STATUS MdtpDisable (VOID)
{
  BOOLEAN MdtpActive = FALSE;
  EFI_STATUS Status = EFI_SUCCESS;
  QCOM_MDTP_PROTOCOL *MdtpProtocol;

  if (FixedPcdGetBool (EnableMdtpSupport)) {
    Status = IsMdtpActive (&MdtpActive);

    if (EFI_ERROR (Status))
      return Status;

    if (MdtpActive) {
      Status = gBS->LocateProtocol (&gQcomMdtpProtocolGuid, NULL,
                                    (VOID **)&MdtpProtocol);
      if (EFI_ERROR (Status)) {
        DEBUG ((EFI_D_ERROR, "Failed to locate MDTP protocol, Status=%r\n",
                Status));
        return Status;
      }
      /* Perform Local Deactivation of MDTP */
      Status = MdtpProtocol->MdtpDeactivate (MdtpProtocol, FALSE);
    }
  }

  return Status;
}

STATIC UINT8
GetRebootReason (UINT32 *ResetReason)
{
  EFI_RESETREASON_PROTOCOL *RstReasonIf;
  EFI_STATUS Status;

  Status = gBS->LocateProtocol (&gEfiResetReasonProtocolGuid, NULL,
                                (VOID **)&RstReasonIf);
  if (Status != EFI_SUCCESS) {
    DEBUG ((EFI_D_ERROR, "Error locating the reset reason protocol\n"));
    return Status;
  }

  RstReasonIf->GetResetReason (RstReasonIf, ResetReason, NULL, NULL);
  if (RstReasonIf->Revision >= EFI_RESETREASON_PROTOCOL_REVISION)
    RstReasonIf->ClearResetReason (RstReasonIf);
  return Status;
}

/**
  Linux Loader Application EntryPoint

  @param[in] ImageHandle    The firmware allocated handle for the EFI image.
  @param[in] SystemTable    A pointer to the EFI System Table.

  @retval EFI_SUCCESS       The entry point is executed successfully.
  @retval other             Some error occurs when executing this entry point.

 **/

EFI_STATUS EFIAPI  __attribute__ ( (no_sanitize ("safe-stack")))
LinuxLoaderEntry (IN EFI_HANDLE ImageHandle, IN EFI_SYSTEM_TABLE *SystemTable)
{
  EFI_STATUS Status;

  UINT32 BootReason = NORMAL_MODE;
  UINT32 KeyPressed;
  /* MultiSlot Boot */
  BOOLEAN MultiSlotBoot;

  DEBUG ((EFI_D_INFO, "Loader Build Info: %a %a\n", __DATE__, __TIME__));
  DEBUG ((EFI_D_VERBOSE, "LinuxLoader Load Address to debug ABL: 0x%llx\n",
         (UINTN)LinuxLoaderEntry & (~ (0xFFF))));
  DEBUG ((EFI_D_VERBOSE, "LinuxLoaderEntry Address: 0x%llx\n",
         (UINTN)LinuxLoaderEntry));

  Status = AllocateUnSafeStackPtr ();
  if (Status != EFI_SUCCESS) {
    DEBUG ((EFI_D_ERROR, "Unable to Allocate memory for Unsafe Stack: %r\n",
            Status));
    goto stack_guard_update_default;
  }
  gST1 = SystemTable; 
  gBS1 = gST1->BootServices; 
  gBS1->LocateProtocol(
      &gEfiGraphicsOutputProtocolGuid, NULL, (VOID **)&mGop);


  // Prepare LittleVGL
  lv_init();   
  static lv_disp_buf_t disp_buf;
  static lv_color_t buf[LV_HOR_RES_MAX * 10]; /*Declare a buffer for 10 lines*/
  lv_disp_buf_init( & disp_buf, buf, NULL, LV_HOR_RES_MAX * 10); /*Initialize the display buffer*/
  lv_disp_drv_t disp_drv; /*Descriptor of a display driver*/
  lv_disp_drv_init( & disp_drv); /*Basic initialization*/
  disp_drv.flush_cb =  EfiGopBltFlush; /*Set your driver function*/  
  disp_drv.buffer = & disp_buf; /*Assign the buffer to the display*/
  lv_disp_drv_register( & disp_drv); /*Finally register the driver*/

  lv_obj_t * win = lv_win_create(lv_scr_act(), NULL);
  lv_win_set_title(win, "Boot Menu"); 
  lv_obj_t * list1 = lv_list_create(win, NULL);
  lv_obj_set_size(list1, 1000, 1050);

  lv_group_t * g1 = lv_group_create();
  lv_group_add_obj(g1, list1);
  lv_group_focus_obj(list1);


  lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);      /*Basic initialization*/
  indev_drv.type = LV_INDEV_TYPE_KEYPAD;
  indev_drv.read_cb = key_read;
  /*Register the driver in LVGL and save the created input device object*/
  lv_indev_t * my_indev = lv_indev_drv_register(&indev_drv);
  lv_indev_set_group(my_indev, g1);

  lv_obj_t * list_btn;
  lv_obj_set_state(list1, LV_STATE_DEFAULT);
  list_btn = lv_list_add_btn(list1,  LV_SYMBOL_FILE, "Example 1");
  list_btn = lv_list_add_btn(list1,  LV_SYMBOL_FILE, "Example 2");
  list_btn = lv_list_add_btn(list1,  LV_SYMBOL_FILE, "Extras");
  lv_list_set_anim_time(list1, 500);
int i=0;
  while (i<1000) {
    lv_tick_inc(1);
    lv_task_handler();
    gBS1->Stall(EFI_TIMER_PERIOD_MILLISECONDS(1));
i++;
  }

  StackGuardChkSetup ();

  BootStatsSetTimeStamp (BS_BL_START);

  // Initialize verified boot & Read Device Info
  Status = DeviceInfoInit ();
  if (Status != EFI_SUCCESS) {
    DEBUG ((EFI_D_ERROR, "Initialize the device info failed: %r\n", Status));
    goto stack_guard_update_default;
  }

  Status = EnumeratePartitions ();

  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_ERROR, "LinuxLoader: Could not enumerate partitions: %r\n",
            Status));
    goto stack_guard_update_default;
  }

  UpdatePartitionEntries ();
  /*Check for multislot boot support*/
  MultiSlotBoot = PartitionHasMultiSlot ((CONST CHAR16 *)L"boot");
  if (MultiSlotBoot) {
    DEBUG ((EFI_D_VERBOSE, "Multi Slot boot is supported\n"));
    FindPtnActiveSlot ();
  }

  Status = GetKeyPress (&KeyPressed);
  if (Status == EFI_SUCCESS) {
    if (KeyPressed == SCAN_DOWN)
      BootIntoFastboot = TRUE;
    if (KeyPressed == SCAN_UP)
      BootIntoRecovery = TRUE;
    if (KeyPressed == SCAN_ESC)
      RebootDevice (EMERGENCY_DLOAD);
  } else if (Status == EFI_DEVICE_ERROR) {
    DEBUG ((EFI_D_ERROR, "Error reading key status: %r\n", Status));
    goto stack_guard_update_default;
  }

  // check for reboot mode
  Status = GetRebootReason (&BootReason);
  if (Status != EFI_SUCCESS) {
    DEBUG ((EFI_D_ERROR, "Failed to get Reboot reason: %r\n", Status));
    goto stack_guard_update_default;
  }

  switch (BootReason) {
  case FASTBOOT_MODE:
    BootIntoFastboot = TRUE;
    break;
  case RECOVERY_MODE:
    BootIntoRecovery = TRUE;
    break;
  case ALARM_BOOT:
    BootReasonAlarm = TRUE;
    break;
  case DM_VERITY_ENFORCING:
    // write to device info
    Status = EnableEnforcingMode (TRUE);
    if (Status != EFI_SUCCESS)
      goto stack_guard_update_default;
    break;
  case DM_VERITY_LOGGING:
    /* Disable MDTP if it's Enabled through Local Deactivation */
    Status = MdtpDisable ();
    if (EFI_ERROR (Status) && Status != EFI_NOT_FOUND) {
      DEBUG ((EFI_D_ERROR, "MdtpDisable Returned error: %r\n", Status));
      goto stack_guard_update_default;
    }
    // write to device info
    Status = EnableEnforcingMode (FALSE);
    if (Status != EFI_SUCCESS)
      goto stack_guard_update_default;

    break;
  case DM_VERITY_KEYSCLEAR:
    Status = ResetDeviceState ();
    if (Status != EFI_SUCCESS) {
      DEBUG ((EFI_D_ERROR, "VB Reset Device State error: %r\n", Status));
      goto stack_guard_update_default;
    }
    break;
  default:
    if (BootReason != NORMAL_MODE) {
      DEBUG ((EFI_D_ERROR,
             "Boot reason: 0x%x not handled, defaulting to Normal Boot\n",
             BootReason));
    }
    break;
  }

  Status = RecoveryInit (&BootIntoRecovery);
  if (Status != EFI_SUCCESS)
    DEBUG ((EFI_D_VERBOSE, "RecoveryInit failed ignore: %r\n", Status));

  /* Populate board data required for fastboot, dtb selection and cmd line */
  Status = BoardInit ();
  if (Status != EFI_SUCCESS) {
    DEBUG ((EFI_D_ERROR, "Error finding board information: %r\n", Status));
    return Status;
  }

  if (!BootIntoFastboot) {
    BootInfo Info = {0};
    Info.MultiSlotBoot = MultiSlotBoot;
    Info.BootIntoRecovery = BootIntoRecovery;
    Info.BootReasonAlarm = BootReasonAlarm;
    Status = LoadImageAndAuth (&Info);
    if (Status != EFI_SUCCESS) {
      DEBUG ((EFI_D_ERROR, "LoadImageAndAuth failed: %r\n", Status));
      goto fastboot;
    }

    BootLinux (&Info);
  }

fastboot:
  DEBUG ((EFI_D_INFO, "Launching fastboot\n"));
  Status = FastbootInitialize ();
  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_ERROR, "Failed to Launch Fastboot App: %d\n", Status));
    goto stack_guard_update_default;
  }

stack_guard_update_default:
  /*Update stack check guard with defualt value then return*/
  __stack_chk_guard = DEFAULT_STACK_CHK_GUARD;
  return Status;
}
