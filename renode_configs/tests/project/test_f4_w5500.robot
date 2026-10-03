*** Settings ***
Suite Setup         Setup F4 W5500
Suite Teardown      Teardown
Test Setup          No Operation
Test Teardown       No Operation
Test Timeout        120s
Library             OperatingSystem

*** Keywords ***
Setup F4 W5500
    Setup
    ${W5500_CS}=    Evaluate    os.path.abspath("${CURDIR}/../../peripherals/W5500.cs")    modules=os
    ${BOARD}=      Evaluate    os.path.abspath("${CURDIR}/../../platforms/boards/stm32f4_w5500.repl")    modules=os
    Execute Command  include "${W5500_CS}"
    Execute Command  mach create "f4w5500"
    Execute Command  machine LoadPlatformDescription "${BOARD}"
    Execute Command  mach set "f4w5500"; start

*** Test Cases ***
Machine Boots
    [Tags]    smoke
    Execute Command  mach set "f4w5500"
    Execute Command  sysbus ReadDoubleWord 0x08000000

W5500 Attached To SPI1
    [Tags]    smoke
    Execute Command  mach set "f4w5500"
    ${tree}=  Execute Command  peripherals
    Should Contain  ${tree}  w5500

SPI1 Registers Accessible
    [Tags]    smoke
    Execute Command  mach set "f4w5500"
    Execute Command  sysbus ReadDoubleWord 0x40013000

W5500 Datagram Fidelity
    [Tags]    model
    ${SCRIPT}=    Evaluate    os.path.abspath("${CURDIR}/../../../scripts/test_w5500_model.sh")    modules=os
    ${rc}    ${out}=    Run And Return Rc And Output    bash "${SCRIPT}"
    Log    ${out}
    Should Be Equal As Integers    ${rc}    0
    Should Contain    ${out}    ALL TESTS PASSED
