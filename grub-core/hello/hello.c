/* hello.c - test module for dynamic loading */
/*
 *  GRUB  --  GRand Unified Bootloader
 *  Copyright (C) 2003,2007  Free Software Foundation, Inc.
 *  Copyright (C) 2003  NIIBE Yutaka <gniibe@m17n.org>
 *
 *  GRUB is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  GRUB is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with GRUB.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <grub/types.h>
#include <grub/misc.h>
#include <grub/mm.h>
#include <grub/err.h>
#include <grub/dl.h>
#include <grub/extcmd.h>
#include <grub/i18n.h>

#ifdef GRUB_MACHINE_EFI
#include <grub/efi/api.h>
#include <grub/efi/efi.h>
#endif

GRUB_MOD_LICENSE ("GPLv3+");

static grub_err_t
grub_cmd_hello (grub_extcmd_context_t ctxt __attribute__ ((unused)),
		int argc __attribute__ ((unused)),
		char **args __attribute__ ((unused)))
{
  grub_printf ("%s\n", _("Hello World"));

#ifdef GRUB_MACHINE_EFI
  grub_efi_uint16_t *os_boot_current = NULL;
  grub_size_t oi_size = 0;
  grub_efi_guid_t global = GRUB_EFI_GLOBAL_VARIABLE_GUID;

  os_boot_current = grub_efi_get_variable("BootCurrent", &global, &oi_size);
  if(os_boot_current == NULL) {
        grub_printf("%s\n", _("grub_efi_get_variable BootCurrent is null"));
  }
  else
  {
       grub_printf(_("efi_variable BootCurrent data is 0x%04x, datasize is 0x2\n"), *os_boot_current);
  }
  /*add start by ning.pan*/
  grub_efi_uint8_t *os_recovery = NULL;
  grub_size_t oi_size1 = 0;
  grub_efi_guid_t global1 = ADV_RECOVERY_KEY_GUID;

  os_recovery = grub_efi_get_variable("RecoveryButton", &global1, &oi_size1);

  if(os_recovery == NULL) {
        grub_printf("%s\n",_("grub_efi_get_variable os_recovery is null"));
  }
  else
  {
       grub_printf(_("efi_variable RecoveryButton data is 0x%02x,datasize is 0x1\n"), *os_recovery);
  }
#endif

  return 0;
}

static grub_extcmd_t cmd;

GRUB_MOD_INIT(hello)
{
  cmd = grub_register_extcmd ("hello", grub_cmd_hello, 0, 0,
			      N_("Say `Hello World'."), 0);
}

GRUB_MOD_FINI(hello)
{
  grub_unregister_extcmd (cmd);
}
