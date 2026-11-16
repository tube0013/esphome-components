from component_paths import COMPONENTS_ROOT

def test_native_entry_modes_and_transfer_gate():
    source=(COMPONENTS_ROOT / 'efr32_flasher/efr32_flasher.cpp').read_text()
    update=source.split('void EFR32Flasher::run_update_(){',1)[1].split('void EFR32Flasher::run_check_update_',1)[0]
    gate=update.split('if (!enter_bootloader_()) {',1)[1].split('ESP_LOGD(TAG, "Bootloader entered',1)[0]
    assert 'esp_http_client_cleanup(client)' in gate and 'set_busy_(false)' in gate and 'return;' in gate
    assert update.index('if (!enter_bootloader_())') < update.index('xmodem_send_(')
    entry=source.split('bool EFR32Flasher::enter_bootloader_(){',1)[1].split('void EFR32Flasher::leave_bootloader_',1)[0]
    software=entry.split('if (!bl_sw_ || !rst_sw_)',1)[0]
    assert software.index('apply_runtime_baud_()') < software.index('ezsp_bootloader::launch(')
    assert 'probe_bootloader_menu_(4000)' in software
    assert 'bl_sw_->' not in software



def test_older_uart_configuration():
    # Reconfiguring older UARTs must never claim MG24's RTS/CTS or TX/RX pins.
    source=(COMPONENTS_ROOT / 'efr32_flasher/efr32_flasher.cpp').read_text()
    assert 'uart_set_pin(port, 4, 36' not in source
    assert 'if (hardware_flow_control_) uart_set_pin(port, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, 2, 13)' in source
    assert 'variant_force_ == 3 && !doc["variants"].isNull()' in source
