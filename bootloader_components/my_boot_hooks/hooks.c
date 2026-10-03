#include <esp_log.h>
#include <rom/rtc.h>
#include <esp_rom_sys.h>
#pragma message "BOOT HOOK"
/* Function used to tell the linker to include this file
 * with all its symbols.
 */
void bootloader_hooks_include(void) { }

void bootloader_before_init(void) {
    //ESP_EARLY_LOGI("boot", __FUNCTION__);
    int reason = esp_rom_get_reset_reason(PRO_CPU_NUM);
    if(reason == RESET_REASON_CHIP_POWER_ON ||
		reason == RESET_REASON_SYS_BROWN_OUT ||
		reason == RESET_REASON_SYS_CLK_GLITCH) {
        esp_rom_delay_us(500000); //esp_rom_get_reset_reason(PRO_CPU_NUM)
    }
    //ESP_EARLY_LOGW("HOOK", "Reset reason 0x%X", reason);
    /* Keep in my mind that a lot of functions cannot be called from here
     * as system initialization has not been performed yet, including
     * BSS, SPI flash, or memory protection. */
    //ESP_LOGI("HOOK", "BEFORE bootloader initialization");
}

void bootloader_after_init(void) {
    //ESP_EARLY_LOGW("HOOK", "bootloader inited");
}
