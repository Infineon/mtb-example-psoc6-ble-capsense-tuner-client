# PSoC&trade; 6 MCU: Tuning CapSense&trade; over Bluetooth&reg; LE  - client

This code example demonstrates how to monitor the CapSense&trade; data and tune CapSense&trade; sensors over Bluetooth&reg; LE communication with CapSense&trade; tuner GUI using PSoC™ 6 CY8C63x7 MCU with AIROC™ Bluetooth® LE.

## Overview

This code example has two applications:

1. **Tuning CapSense&trade; over Bluetooth&reg; LE - server:** This design has a self-capacitance (CSD) based 5-segment CapSense&trade; slider and two mutual capacitance (CSX)-based CapSense&trade; buttons. It has a PSoC™ 6 CY8C63x7 MCU with AIROC™ Bluetooth® LE configured as a Bluetooth&reg; LE Peripheral and GATT Server that sends CapSense&trade; raw data to the GATT Client. This project should be run on a board with CapSense&trade; sensors that needs to be tuned over Bluetooth&reg; LE communication.

2. **Tuning CapSense&trade; over Bluetooth&reg; LE - client:** The design has a PSoC™ 6 CY8C63x7 MCU with AIROC™ Bluetooth® LE configured as a GATT Client to receive CapSense&trade; raw data from a GATT Server. It also has an EZI2C interface to communicate with the CapSense&trade; tuner. This project needs to be run on a board that has Bluetooth&reg; LE capability and has a connection to the CapSense&trade; tuner through the EZI2C interface.

**Note:** This code example shows the GATT Client-side of the example project. See [PSoC™ 6 MCU: tuning CapSense&trade; over Bluetooth&reg; LE- server](https://github.com/cypresssemiconductorco/mtb-example-psoc6-ble-capsense-tuner-server) for the GATT Server-side of the project.

**Figure 1. High-level block diagram**

![](images/overall-block-diag.png)

[Provide feedback on this code example.](https://cypress.co1.qualtrics.com/jfe/form/SV_1NTns53sK2yiljn?Q_EED=eyJVbmlxdWUgRG9jIElkIjoiQ0UyMzAyMjkiLCJTcGVjIE51bWJlciI6IjAwMi0zMDIyOSIsIkRvYyBUaXRsZSI6IlBTb0MmdHJhZGU7IDYgTUNVOiBUdW5pbmcgQ2FwU2Vuc2UmdHJhZGU7IG92ZXIgQmx1ZXRvb3RoJnJlZzsgTEUgIC0gY2xpZW50IiwicmlkIjoiYnJ2aSIsIkRvYyB2ZXJzaW9uIjoiMi4xLjAiLCJEb2MgTGFuZ3VhZ2UiOiJFbmdsaXNoIiwiRG9jIERpdmlzaW9uIjoiTUNEIiwiRG9jIEJVIjoiSUNXIiwiRG9jIEZhbWlseSI6IlBTT0MifQ==)

## Requirements

- [ModusToolbox&trade; software](https://www.cypress.com/products/modustoolbox-software-environment) v2.2 or later

   **Note:** This code example version requires ModusToolbox&trade; software version 2.2 or later and is not backward compatible with v2.1 or older versions. If you cannot move to ModusToolbox&trade; v2.2, use the latest compatible version of this example: [latest-v1.X](https://github.com/cypresssemiconductorco/mtb-example-psoc6-ble-capsense-tuner-client/tree/latest-v1.X).

- Board support package (BSP) minimum required version: 2.0.0
- Programming language: C
- Associated parts: All [PSoC&trade; 6 MCU](http://www.cypress.com/PSoC6) parts

## Supported toolchains (make variable 'TOOLCHAIN')

- GNU Arm® embedded compiler v9.3.1 (`GCC_ARM`) - Default value of `TOOLCHAIN`
- Arm&reg; compiler v6.13 (`ARM`)
- IAR C/C++ compiler v8.42.2 (`IAR`)

## Supported kits (make variable 'TARGET')

### Tuning CapSense&trade; over Bluetooth&reg; LE - server

- [PSoC&trade; 6 Bluetooth&reg; LE pioneer kit](https://www.cypress.com/CY8CKIT-062-BLE) (`CY8CKIT-062-BLE`) - Default value of `TARGET`

### Tuning CapSense&trade; over Bluetooth&reg; LE - client

- [PSoC&trade; 6 Bluetooth&reg; LE pioneer kit](https://www.cypress.com/CY8CKIT-062-BLE) (CY8CKIT-062-BLE) - Default value of `TARGET`
- [PSoC&trade; 6 Bluetooth&reg; LE prototyping kit](https://www.cypress.com/CY8CPROTO-063-BLE) (CY8CPROTO-063-BLE)
- [EZ-BLE Arduino evaluation board](https://www.cypress.com/CYBLE-416045-EVAL) (CYBLE-416045-EVAL)

## Hardware setup

This example uses the board's default configuration. See the kit user guide to ensure that the board is configured correctly.

Two PSoC&trade; 6 Bluetooth&reg; LE boards are required to test this code example: one for the application **Tuning CapSense&trade; over Bluetooth&reg; LE - server** and another for the application **Tuning CapSense&trade; over Bluetooth&reg; LE - client**. See [Supported kits](#supported-kits-make-variable-target) for the list of kits supported by each code example.

**Note:** The PSoC&trade; 6 Bluetooth&reg; LE pioneer kit (CY8CKIT-062-BLE) and the PSoC&trade; 6 Wi-Fi Bluetooth&reg; pioneer kit (CY8CKIT-062-WIFI-BT) ship with KitProg2 installed. The ModusToolbox&trade; software requires KitProg3. Before using this code example, make sure that the board is upgraded to KitProg3. The tool and instructions are available in the [Firmware Loader](https://github.com/cypresssemiconductorco/Firmware-loader) GitHub repository. If you do not upgrade, you will see an error like "unable to find CMSIS-DAP device" or "KitProg firmware is out of date".

## Software setup

Install a terminal emulator if you don't have one. Instructions in this document use [Tera Term](https://ttssh2.osdn.jp/index.html.en).

This example requires no additional software or tools.

## Using the code example

Create the project and open it using one of the following:

<details><summary><b>In Eclipse IDE for ModusToolbox&trade;</b></summary>

1. Click the **New Application** link in the **Quick Panel** (or, use **File** > **New** > **ModusToolbox Application**). This launches the [Project Creator](http://www.cypress.com/ModusToolboxProjectCreator) tool.

2. Pick a kit supported by the code example from the list shown in the **Project Creator - Choose Board Support Package (BSP)** dialog.

   When you select a supported kit, the example is reconfigured automatically to work with the kit. To work with a different supported kit later, use the [Library manager](https://www.cypress.com/ModusToolboxLibraryManager) to choose the BSP for the supported kit. You can use the Library Manager to select or update the BSP and firmware libraries used in this application. To access the Library manager, click the link from the **Quick Panel**.

   You can also just start the application creation process again and select a different kit.

   If you want to use the application for a kit not listed here, you may need to update the source files. If the kit does not have the required resources, the application may not work.

3. In the **Project Creator - Select Application** dialog, choose the examples **CapSense Tuning Over BLE - Server** and **CapSense Tuning Over BLE - Client** by enabling the checkbox.

4. Optionally, change the suggested **New Application Name**.

5. Enter the local path in the **Application(s) Root Path** field to indicate where the application needs to be created.

   Applications that can share libraries can be placed in the same root path.

6. Click **Create** to complete the application creation process.

For more details, see the [Eclipse IDE for ModusToolbox&trade; user guide](https://www.cypress.com/MTBEclipseIDEUserGuide) (locally available at *{ModusToolbox&trade; install directory}/ide_{version}/docs/mt_ide_user_guide.pdf*).

</details>

<details><summary><b>In command-line interface (CLI)</b></summary>

ModusToolbox&trade; provides the Project Creator as both a GUI tool and a command line tool to easily create one or more ModusToolbox&trade; applications. See the "Project Creator tools" section of the [ModusToolbox&trade; user guide](https://www.cypress.com/ModusToolboxUserGuide) for more details.

Alternatively, you can manually create the application using the following steps:

1. Download and unzip this repository onto your local machine, or clone the repository.

2. Open a CLI terminal and navigate to the application folder.

   On Windows, use the command line "modus-shell" program provided in the ModusToolbox&trade; installation instead of a standard Windows command line application. This shell provides access to all ModusToolbox&trade; tools. You can access it by typing `modus-shell` in the search box in the Windows menu.

   In Linux and macOS, you can use any terminal application.

   **Note:** The cloned application contains a default BSP file (*TARGET_xxx.mtb*) in the *deps* folder. Use the [Library manager](https://www.cypress.com/ModusToolboxLibraryManager) (`make modlibs` command) to select and download a different BSP file, if required. If the selected kit does not have the required resources or is not [supported](#supported-kits-make-variable-target), the application may not work.

3. Import the required libraries by executing the `make getlibs` command.

4. Repeat steps 1 to 3 for the repository [PSoC&trade; 6 MCU: tuning CapSense&trade; over Bluetooth&reg; LE - server](https://github.com/cypresssemiconductorco/mtb-example-psoc6-ble-capsense-tuner-client) and place it in the same workspace folder.

Various CLI tools include a `-h` option that prints help information to the terminal screen about that tool. For more details, see the [ModusToolbox&trade; user guide](https://www.cypress.com/ModusToolboxUserGuide) (locally available at *{ModusToolbox&trade; install directory}/docs_{version}/mtb_user_guide.pdf*).

</details>

<details><summary><b>In third-party IDEs</b></summary>

1. Follow the instructions from the **In command-line interface (CLI)** section to create the application, and import the libraries using the `make getlibs` command.

2. Export the application to a supported IDE using the `make <ide>` command.

   For a list of supported IDEs and more details, see the "Exporting to IDEs" section of the [ModusToolbox&trade; user guide](https://www.cypress.com/ModusToolboxUserGuide) (locally available at *{ModusToolbox&trade; install directory}/docs_{version}/mtb_user_guide.pdf*.

3. Follow the instructions displayed in the terminal to create or import the application as an IDE project.

</details>

## Operation

### Tuning CapSense&trade; over Bluetooth&reg; LE - server

See [Supported kits](#supported-kits-make-variable-target) for the list of kits supported by this code example.

1. Connect the board to your PC using the provided USB cable through the KitProg3 USB connector.

2. Open a terminal program and select the KitProg3 COM port. Set the serial port parameters to 8N1 and 115200 baud.

3. Program the board using one of the following:

   <details><summary><b>Using Eclipse IDE for ModusToolbox&trade;</b></summary>

      1. Select the application project **Tuning CapSense&trade; over Bluetooth&reg; LE - server** in the Project Explorer.

      2. In the **Quick Panel**, scroll down, and click **\<Application Name> Program (KitProg3_MiniProg4)**.
      </details>

   <details><summary><b>Using CLI</b></summary>

     From the terminal, execute the `make program` command to build and program the application using the default toolchain to the default target. You can specify a target and toolchain manually:
      ```
      make program TARGET=<BSP> TOOLCHAIN=<toolchain>
      ```

      Example:
      ```
      make program TARGET=CY8CKIT-062-BLE TOOLCHAIN=GCC_ARM
      ```
   </details>

4. After programming, the application starts automatically. Confirm that "Tuning CapSense over BLE - Server" is displayed on the UART terminal.

**Figure 2. Terminal output of the GATT Server while advertising**

![](images/server-advertise.png)

### Tuning CapSense&trade; over Bluetooth&reg; LE - client

See [Supported kits](#supported-kits-make-variable-target) for the list of kits supported by this project.

1. Connect the board to your PC using the provided USB cable through the KitProg3 USB connector.

2. Open a terminal program and select the KitProg3 COM port. Set the serial port parameters to 8N1 and 115200 baud.

3. Program the board using one of the following:

   <details><summary><b>Using Eclipse IDE for ModusToolbox&trade;</b></summary>

    1. Select the application project **Tuning CapSense&trade; over Bluetooth&reg; LE - client** in the Project Explorer.

    2. In the **Quick Panel**, scroll down, and click **\<Application Name> Program (KitProg3_MiniProg4)**.

   </details>

   <details><summary><b>Using CLI</b></summary>

     From the terminal, execute the `make program` command to build and program the application using the default toolchain to the default target. You can specify a target and toolchain manually:
      ```
      make program TARGET=<BSP> TOOLCHAIN=<toolchain>
      ```

      Example:
      ```
      make program TARGET=CY8CPROTO-063-BLE TOOLCHAIN=GCC_ARM
      ```
   </details>

4. After programming, the application starts automatically. Confirm that "Tuning CapSense over BLE - Client" is displayed on the UART terminal.

**Figure 3. Terminal output of the GATT Client while scanning**

![](images/client-scan.png)

### Establishing a Bluetooth&reg; LE connection

1. Connect both boards to your PC and open the corresponding terminals to view UART messages. If required, reset the board after opening the UART terminal.

2. Observe that the USER_LED1 is turned ON in both boards indicating that the Bluetooth&reg; LE connection is established between the two PSoC&trade; 6 MCU devices.

3. Verify that the tuner bridge initialization data is sent from the GATT Server to the GATT Client. See Figure 4 and Figure 5.

**Figure 4. Terminal output of the GATT Server**

![](images/server-tuner-init.png)

<br>

**Figure 5. Terminal output of the GATT Client**

![](images/client-tuner-init.png)

### Monitoring CapSense&trade; data, tuning the parameters, and measuring the SNR

1. Select **File** > **Disconnect** and disconnect the UART terminal connected to the device programmed with **Tuning CapSense&trade; over Bluetooth&reg; LE - client**.

2. Launch the CapSense&trade; tuner for the **Tuning CapSense&trade; over Bluetooth&reg; LE - server** application. See the "Launch the CapSense&trade; tuner" section of the [CapSense&trade; tuner guide](https://www.cypress.com/file/504381/download).

3. In CapSense&trade; tuner, click the **Setup Tuner Communication** button. Alternatively, select **Tools** > **Setup Tuner Communication**.

4. Connect to the KitProg of the kit that is programmed with **Tuning CapSense&trade; over Bluetooth&reg; LE - client**. Check the **I2C** checkbox and set the following values in the port configuration section:

   * *I2C Address* : 8
   * *Sub-address* : 2-Bytes
   * *Speed (kHz)* : 1000

5. Click **Connect**, and then click **Start** to extract the CapSense&trade; data. You can see the data rate of the packets in the bottom panel.

6. Touch the sensors on the board that is programmed with **Tuning CapSense&trade; over Bluetooth&reg; LE - server** and notice the change in the sensor/widget status on the **Widget View** tab.

7. Open the **Graph View** tab. Check the sensors in the **Widget Explorer** pane to observe sensor signals in the graph. Touch the sensors and notice the signal change on the **Graph View** tab.

8. Change widget/sensor parameter values as needed, and then click **Apply to Device** to apply the new settings to the device.

   **Note:** The **Apply to Device** option requires the CapSense&trade; tuner to be in Synchronized mode of operation.

For more details on monitoring CapSense&trade; data, tuning CapSense&trade; parameter tuning, and measuring SNR, see the CapSense&trade; tuner guide. See [AN85951 – PSoC&trade; 4 and PSoC&trade; 6 MCU CapSense&trade; design guide](https://www.cypress.com/documentation/application-notes/an85951-psoc-4-and-psoc-6-mcu-capsense-design-guide) for more details on selecting the right tuning parameters.

## Debugging

You can debug the example to step through the code. In the IDE, use the **\<Application Name> Debug (KitProg3_MiniProg4)** configuration in the **Quick Panel**. For more details, see the "Program and debug" section in the [Eclipse IDE for ModusToolbox&trade; user guide](https://www.cypress.com/MTBEclipseIDEUserGuide).

**Note:** **(Only while debugging)** On the CM4 CPU, some code in `main()` may execute before the debugger halts at the beginning of `main()`. This means that some code executes twice - once before the debugger stops execution, and again after the debugger resets the program counter to the beginning of `main()`. See [KBA231071](https://community.cypress.com/docs/DOC-21143) to learn about this and for the workaround.

## Design and implementation

### Tuning CapSense&trade; over Bluetooth&reg; LE - server

The design has a PSoC™ 6 CY8C63x7 MCU with AIROC™ Bluetooth® LE device configured as a GAP Peripheral and a GATT Server with the *CapSense_Tuner* custom service. This service has two custom characteristics: *CapSense_DS* and *Tuner_Command*. The *CapSense_DS* characteristic is loaded with the CapSense&trade; context structure *cy_capsense_tuner*. The *Tuner_Command* characteristic is used to receive command packets from the GATT Client which were received from the CapSense&trade; tuner. This code example supports 2M PHY and data length extension (DLE) features to maximize the throughput.

The design also has a CSD-based, 5-segment CapSense&trade; slider and two CSX-based CapSense&trade; buttons. The project uses the CapSense&trade; middleware. See [ModusToolbox&trade; user guide](https://www.cypress.com/file/504361/download) for more details on selecting a middleware. See [AN85951 – PSoC&trade; 4 and PSoC&trade; 6 MCU CapSense&trade; design guide](https://www.cypress.com/documentation/application-notes/an85951-psoc-4-and-psoc-6-mcu-capsense-design-guide) for more details of CapSense&trade; features and usage.

#### Firmware flow

When the kit is powered up, the CapSense&trade; and Bluetooth&reg; LE blocks are initialized. The PSoC™ 6 CY8C63x7 MCU with AIROC™ Bluetooth® LE device is configured as Bluetooth&reg; LE GAP Peripheral and GATT Server. The Bluetooth&reg; LE initialization function initializes the Bluetooth&reg; LE stack and registers the Bluetooth&reg; LE stack event handler (an application-level callback function to process Bluetooth&reg; LE events). The *tuner_send_callback* function (an application-level callback function to facilitate data transmission using Bluetooth&reg; LE) is registered in the CapSense&trade; middleware to send the *cy_capsense_tuner* CapSense&trade; context structure to the GATT Client.

When the Bluetooth&reg; LE connection to the GATT Client device is successful, USER_LED1 will turn ON.

After a successful Bluetooth&reg; LE connection, the GATT Client enables the notifications of the *CapSense_DS* characteristic using client characteristic configuration descriptor (CCCD). Once notifications are enabled, tuner bridge initialization parameters such as the size of CapSense&trade; context structure and the number of notification packets required to send the complete CapSense&trade; context structure are sent to the peer GATT Client device.

The application periodically scans the CapSense&trade; buttons for user inputs. The `Cy_CapSense_RunTuner()` function is called periodically in the application program to establish synchronized communication with the CapSense&trade; tuner application. The `Cy_CapSense_RunTuner()` function calls the user-registered callback function `tuner_send_callback` to send the CapSense&trade; data structure `cy_capsense_tuner`  to the GATT Client. The `cy_capsense_tuner` structure is sent as notification packets using the *CapSense_DS* characteristic.

When you change the CapSense&trade; hardware parameters such as resolution, number of sub-conversions, and so on from the CapSense&trade; tuner, it modifies the CapSense&trade; context structure. The GATT Server receives this as a write command through the *Tuner_Command* characteristic. The write command contains the offset address of the CapSense&trade; context structure that is modified, actual data modified, and the number of bytes modified by the CapSense&trade; tuner. The application is notified of this event through the Bluetooth&reg; LE stack event handler. The application then modifies the CapSense&trade; context structure directly using this information.

**Figure 6. High-level firmware flowchart**

![](images/server-flowchart.png)

#### **Resources and settings**

**Table 1. Application resources**

| Resource  |  Alias/object     |    Purpose     |
| :------- | :------------    | :------------ |
| Bluetooth&reg; LE | CYBSP_BLE | Establish Bluetooth&reg; LE communication with the GATT Client |
| CSD | CYBSP_CSD | CapSense&trade; driver to interface touch sensors |
| UART (HAL)|cy_retarget_io_uart_obj| UART HAL object used by Retarget-IO for debug UART port  |
| GPIO (HAL)    | CYBSP_USER_LED1         | User LED                  |

<br>

### Tuning CapSense&trade; over Bluetooth&reg; LE - client

The design has a PSoC™ 6 CY8C63x7 MCU with AIROC™ Bluetooth® LE device configured as Bluetooth&reg; LE GAP Central and GATT Client with the *CapSense_Tuner* custom service. The design also uses a serial communication block  (SCB) configured as EZIZC to establish I2C communication with the onboard KitProg, which enables the CapSense&trade; tuner to read the CapSense&trade; raw data. The CapSense&trade; tuner is a standalone tool that runs on a PC, and is included in the ModusToolbox&trade; software to tune CapSense&trade; applications. This code example supports 2M PHY and data length extension (DLE) features to maximize the throughput.

#### **Firmware flow**

When the kit is powered up, the Bluetooth&reg; LE and SCB (configured as EZI2C) blocks are initialized. The Bluetooth&reg; LE initialization function initializes the Bluetooth&reg; LE stack and registers the Bluetooth&reg; LE stack event handler (an application-level callback function to process Bluetooth&reg; LE events).

When the Bluetooth&reg; LE connection to the GATT Server device is successful, USER_LED1 will turn ON.

After the Bluetooth&reg; LE connection is established, the notifications are enabled to receive data from the GATT Server using the CCCD handle of the *CapSense_DS* characteristic. Once notifications are enabled, the tuner bridge initialization parameters such as size of CapSense&trade; context structure and the number of notification packets required to send the complete CapSense&trade; context structure are received from the peer GATT Server device. This is used to set the size of the EZI2C buffer.

When you change the CapSense&trade; hardware parameters such as resolution, number of sub-conversions, and so on from the CapSense&trade; tuner, it modifies the CapSense&trade; context structure (EZI2C buffer).

The application does the following:

- Checks whether a notification event is received from the GATT Server device through the *CapSense_DS* characteristic. Once a notification containing the CapSense&trade; context structure is received, it is loaded to the EZI2C buffer to be read by the CapSense&trade; tuner (I2C Master).

- Checks whether a I2C write event has been triggered by the CapSense&trade; tuner (I2C master). Once write event is received, the application finds the offset, number of bytes modified, and the data written on the CapSense&trade; context structure by the CapSense&trade; tuner. This data is communicated to the peer GATT Server through a GATT write command using the *Tuner_Command* characteristic handle.

 **Figure 7. High-level firmware flowchart**

 ![](images/client-flowchart.png)

 #### **Resources and settings**

 **Table 2. Application resources**

| Resource  |  Alias/object     |    Purpose     |
| :------- | :------------    | :------------ |
| Bluetooth&reg; LE | CYBSP_BLE | Establish Bluetooth&reg; LE communication with the GATT Server |
| SCB | EZI2C | EZI2C driver to interface with the CapSense&trade; tuner |
| UART (HAL)|cy_retarget_io_uart_obj| UART HAL object used by Retarget-IO for Debug UART port  |
| GPIO (HAL)    | CYBSP_USER_LED1         | User LED                  |

<br>

## Related resources

| Application notes                                            |                                                              |
| :----------------------------------------------------------- | :----------------------------------------------------------- |
| [AN228571](https://www.cypress.com/AN228571) – Getting started with PSoC&trade; 6 MCU on ModusToolbox&trade;| Describes PSoC&trade; 6 MCU devices and how to build your first application with ModusToolbox&trade;|
| [AN221774](https://www.cypress.com/AN221774) – Getting started with PSoC&trade; 6 MCU on PSoC&trade; Creator&trade; | Describes PSoC&trade; 6 MCU devices and how to build your first application with PSoC&trade; Creator&trade; |
| [AN210781](https://www.cypress.com/AN210781) – Getting started with PSoC&trade; 6 MCU with Bluetooth&reg; Low Energy connectivity on PSoC&trade; Creator&trade; | Describes PSoC&trade; 6 MCU with Bluetooth&reg; LE connectivity devices and how to build your first application with PSoC&trade; Creator&trade; |
| [AN215656](https://www.cypress.com/AN215656) – PSoC&trade; 6 MCU: dual-CPU system design | Describes the dual-CPU architecture in PSoC&trade; 6 MCU, and shows how to build a simple dual-CPU design |
| [AN85951](https://www.cypress.com/AN85951) – PSoC&trade; 4 and PSoC&trade; 6 MCU CapSense&trade; design guide  | Describes how to design capacitive touch sensing applications with the PSoC&trade; 6 MCU families of devices |
| **Code examples**                                            |                                                              |
| [Using ModusToolbox&trade;](https://github.com/cypresssemiconductorco/Code-Examples-for-ModusToolbox-Software) | [Using PSoC&trade; Creator&trade;](https://www.cypress.com/documentation/code-examples/psoc-6-mcu-code-examples) |
| **Device documentation**                                     |                                                              |
| [PSoC&trade; 6 MCU datasheets](https://www.cypress.com/search/all?f[0]=meta_type%3Atechnical_documents&f[1]=resource_meta_type%3A575&f[2]=field_related_products%3A114026) | [PSoC&trade; 6 technical reference manuals](https://www.cypress.com/search/all/PSoC%206%20Technical%20Reference%20Manual?f[0]=meta_type%3Atechnical_documents&f[1]=resource_meta_type%3A583) |
| **Development kits**                                         | Buy at www.cypress.com                                       |
| [CY8CKIT-062-BLE](https://www.cypress.com/CY8CKIT-062-BLE) PSoC&trade; 6 Bluetooth&reg; LE pioneer kit | [CY8CPROTO-063-BLE](https://www.cypress.com/CY8CPROTO-063-BLE) PSoC&trade; 6 Bluetooth&reg; LE prototyping kit |
| [CYBLE-416045-EVAL](https://www.cypress.com/CYBLE-416045-EVAL) EZ-BLE Arduino evaluation board |    |
| **Libraries**                                                 |                                                              |
| PSoC&trade; 6 peripheral driver library (PDL) and docs  | [mtb-pdl-cat1](https://github.com/cypresssemiconductorco/mtb-pdl-cat1) on GitHub |
| Hardware abstraction layer (HAL) Library and docs    | [mtb-hal-cat1](https://github.com/cypresssemiconductorco/mtb-hal-cat1) on GitHub |
| Retarget-IO - A utility library to retarget the standard input/output (STDIO) messages to a UART port | [retarget-io](https://github.com/cypresssemiconductorco/retarget-io) on GitHub |
| **Middleware**                                               |                                                              |
| BLESS library and documents                                       | [bless](https://github.com/cypresssemiconductorco/bless) on GitHub|
| CapSense&trade; library and documents                                   | [capsense](https://github.com/cypresssemiconductorco/capsense) on GitHub |
| Links to all PSoC&trade; 6 MCU middleware                           | [psoc6-middleware](https://github.com/cypresssemiconductorco/psoc6-middleware) on GitHub |
| **Tools**                                                    |                                                              |
| [Eclipse IDE for ModusToolbox&trade;](https://www.cypress.com/modustoolbox) | The cross-platform, Eclipse-based IDE for IoT designers that supports application configuration and development targeting converged MCU and wireless systems. |
| [PSoC&trade; Creator™](https://www.cypress.com/products/psoc-creator-integrated-design-environment-ide) | The legacy Cypress IDE for PSoC&trade; and FM0+ MCU development. |

## Other resources

Cypress provides a wealth of data at www.cypress.com to help you select the right device, and quickly and effectively integrate it into your design.

For PSoC&trade; 6 MCU devices, see [How to design with PSoC&trade; 6 MCU - KBA223067](https://community.cypress.com/docs/DOC-14644) in the Cypress community.

## Document history

Document title: *CE230229* - *PSoC&trade; 6 MCU: Tuning CapSense&trade; over Bluetooth&reg; LE - client*

| Version | Description of Change |
| ------- | --------------------- |
| 1.0.0   | New code example      |
| 2.0.0   | Major update to support ModusToolbox&trade; software v2.2<br> This version is not backward compatible with ModusToolbox&trade; software v2.1  |
| 2.1.0   | Added support for new kit <br>Updated CE to use HAL drivers|
------

All other trademarks or registered trademarks referenced herein are the property of their respective owners.

![banner](images/ifx-cy-banner.png)

-------------------------------------------------------------------------------

© Cypress Semiconductor Corporation, 2016-2021. This document is the property of Cypress Semiconductor Corporation, an Infineon Technologies company, and its affiliates ("Cypress").  This document, including any software or firmware included or referenced in this document ("Software"), is owned by Cypress under the intellectual property laws and treaties of the United States and other countries worldwide.  Cypress reserves all rights under such laws and treaties and does not, except as specifically stated in this paragraph, grant any license under its patents, copyrights, trademarks, or other intellectual property rights.  If the Software is not accompanied by a license agreement and you do not otherwise have a written agreement with Cypress governing the use of the Software, then Cypress hereby grants you a personal, non-exclusive, nontransferable license (without the right to sublicense) (1) under its copyright rights in the Software (a) for Software provided in source code form, to modify and reproduce the Software solely for use with Cypress hardware products, only internally within your organization, and (b) to distribute the Software in binary code form externally to end users (either directly or indirectly through resellers and distributors), solely for use on Cypress hardware product units, and (2) under those claims of Cypress’s patents that are infringed by the Software (as provided by Cypress, unmodified) to make, use, distribute, and import the Software solely for use with Cypress hardware products.  Any other use, reproduction, modification, translation, or compilation of the Software is prohibited.
<br>
TO THE EXTENT PERMITTED BY APPLICABLE LAW, CYPRESS MAKES NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, WITH REGARD TO THIS DOCUMENT OR ANY SOFTWARE OR ACCOMPANYING HARDWARE, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.  No computing device can be absolutely secure.  Therefore, despite security measures implemented in Cypress hardware or software products, Cypress shall have no liability arising out of any security breach, such as unauthorized access to or use of a Cypress product.  CYPRESS DOES NOT REPRESENT, WARRANT, OR GUARANTEE THAT CYPRESS PRODUCTS, OR SYSTEMS CREATED USING CYPRESS PRODUCTS, WILL BE FREE FROM CORRUPTION, ATTACK, VIRUSES, INTERFERENCE, HACKING, DATA LOSS OR THEFT, OR OTHER SECURITY INTRUSION (collectively, "Security Breach").  Cypress disclaims any liability relating to any Security Breach, and you shall and hereby do release Cypress from any claim, damage, or other liability arising from any Security Breach.  In addition, the products described in these materials may contain design defects or errors known as errata which may cause the product to deviate from published specifications.  To the extent permitted by applicable law, Cypress reserves the right to make changes to this document without further notice. Cypress does not assume any liability arising out of the application or use of any product or circuit described in this document.  Any information provided in this document, including any sample design information or programming code, is provided only for reference purposes.  It is the responsibility of the user of this document to properly design, program, and test the functionality and safety of any application made of this information and any resulting product.  "High-Risk Device" means any device or system whose failure could cause personal injury, death, or property damage.  Examples of High-Risk Devices are weapons, nuclear installations, surgical implants, and other medical devices.  "Critical Component" means any component of a High-Risk Device whose failure to perform can be reasonably expected to cause, directly or indirectly, the failure of the High-Risk Device, or to affect its safety or effectiveness.  Cypress is not liable, in whole or in part, and you shall and hereby do release Cypress from any claim, damage, or other liability arising from any use of a Cypress product as a Critical Component in a High-Risk Device.  You shall indemnify and hold Cypress, including its affiliates, and its directors, officers, employees, agents, distributors, and assigns harmless from and against all claims, costs, damages, and expenses, arising out of any claim, including claims for product liability, personal injury or death, or property damage arising from any use of a Cypress product as a Critical Component in a High-Risk Device.  Cypress products are not intended or authorized for use as a Critical Component in any High-Risk Device except to the limited extent that (i) Cypress’s published data sheet for the product explicitly states Cypress has qualified the product for use in a specific High-Risk Device, or (ii) Cypress has given you advance written authorization to use the product as a Critical Component in the specific High-Risk Device and you have signed a separate indemnification agreement.
<br>
Cypress, the Cypress logo, and combinations thereof, WICED, ModusToolBox, PSoC, CapSense, EZ-USB, F-RAM, and Traveo are trademarks or registered trademarks of Cypress or a subsidiary of Cypress in the United States or in other countries.  For a more complete list of Cypress trademarks, visit cypress.com.  Other names and brands may be claimed as property of their respective owners.
