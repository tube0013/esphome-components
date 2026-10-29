"""Host tests for the exact native UART/persistence helpers used in firmware."""
from pathlib import Path
import os
import subprocess
import pytest
ROOT=Path(__file__).resolve().parents[1]


@pytest.mark.parametrize('source,stubs',[
    ('runtime_baud_test.cpp','baud_stubs'),
    ('ezsp_bootloader_test.cpp','ezsp_stubs'),
    ('ezsp_bootloader_uart_test.cpp','ezsp_stubs'),
    ('manifest_baud_test.cpp',None),
    ('operation_start_test.cpp',None),
    ('request_security_test.cpp',None),
])
def test_native_protocol(source,stubs,tmp_path):
    includes=[ROOT]
    if stubs:includes.insert(0,ROOT/'tests'/stubs)
    if source=='ezsp_bootloader_uart_test.cpp':
        namespace=tmp_path/'include/esphome/components';namespace.mkdir(parents=True)
        (namespace/'efr32_info').symlink_to(ROOT/'components/efr32_info',target_is_directory=True)
        includes.append(tmp_path/'include')
    if source=='manifest_baud_test.cpp':
        include=os.environ.get('ARDUINOJSON_INCLUDE')
        if not include:pytest.skip('Set ARDUINOJSON_INCLUDE to ArduinoJson 7.4.3 src (CI does this)')
        includes.append(Path(include))
    exe=tmp_path/source.removesuffix('.cpp')
    cmd=['c++','-std=c++17','-Wall','-Wextra','-Wno-unused-parameter','-Wno-unused-variable']
    for include in includes:cmd.extend(['-I',str(include)])
    subprocess.run(cmd+[str(ROOT/'tests'/source),'-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)


def test_cc2652_member_layout():
    import re
    members=[]
    for suffix in ('h','cpp'):
        source=(ROOT/'components/cc2652_flasher'/('cc2652_flasher.'+suffix)).read_text()
        fields=source.split('esphome::uart::UARTComponent *uart_',1)[1].split('\n};',1)[0]
        members.append(re.sub(r'\s+','',re.sub(r'//[^\n]*','',fields)))
    assert members[0]==members[1]
