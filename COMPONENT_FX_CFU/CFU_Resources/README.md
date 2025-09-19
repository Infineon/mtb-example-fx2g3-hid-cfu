# CFU resources

The resources in this directory aim to assist you to perform CFU with the firmware in this code example.

CFU takes place in two stages:

1. The CFU offer containing the metadata of the upcoming firmware is sent. The device processes and either accepts or rejects the offer

2. On acceptance, the payload is transferred to the device. The payload is the firmware binary, in an SREC-based format, as detailed by the CFU specification

[View this README on GitHub.](https://github.com/Infineon/mtb-example-fx2g3-hid-cfu/COMPONENT_FX_CFU/CFU_Resources/README.md)


## Offer and payload generation

Place the desired input bitfile in this directory and run the following commands by replacing `<bitfile_name>` with the name of selected input bitfile to generate the CFU offer and payload binaries:

> **Note:** Throughout the document, `<bitfile_name>` refers to the name of the initial input bitfile

1. **Clean:**

    ```sh
    rm -f <bitfile_name>.offer.bin <bitfile_name>.payload.bin
    ```

2. **Generate CFU offer and payload binaries:**

    ```sh
    python cfu_generate_offer_and_payload.py <bitfile_name>
    ```

This generates an offer file *<bitfile_name>.offer.bin* and a payload file *<bitfile_name>.payload.bin*.

> **Note:** If you want to utilize specific CFU features with this application (such as __forceIgnoreVersion__), set the appropriate bits in the offer binary file by modifying the corresponding variables in the [offer generation script](./scripts/generate_offer.py#L99). See [Microsoft's CFU implementation guide](https://learn.microsoft.com/en-us/windows-hardware/drivers/cfu/cfu-firmware-implementation-guide#the-offer-and-content-parts) for more details on writing a custom offer binary.


## Microsoft CFU Standalone Tool

Microsoft's CFU Standalone Tool is used for performing the update. The open source tool is available in the [tools section of Microsoft's CFU repository](https://github.com/microsoft/CFU/tree/master/Tools/ComponentFirmwareUpdateStandAloneToolSample).

1. Download the source files from the [CFU repository](https://github.com/microsoft/CFU/tree/master/Tools/ComponentFirmwareUpdateStandAloneToolSample) into this directory and open the *ComponentFirmwareUpdateStandAloneToolSample* directory

    - **Suggested approach:** Clone or download the entire repository and move the *ComponentFirmwareUpdateStandAloneToolSample* folder into this directory

2. Open the Visual Studio Solution file, *CfuExample.sln*, using the [Microsoft Visual Studio IDE](https://visualstudio.microsoft.com/) (v2017 or higher)

3. If prompted, choose to **Trust** the project and wait for the IDE to load the project files

4. If prompted with a version upgrade for the Visual C++ platform toolset, allow the project to retarget to the newer version

5. Click the **Build** button in the IDE's top file Menu bar and choose **Configuration Manager...** to launch it

6. In the **Project contexts** table of the **Configuration Manager**, choose **Release** from the drop-down menu under the **Configuration** column, and **Close** the **Configuration Manager**

7. Right-click **FwUpdateCfu** in the IDE's solution explorer (on the right-side panel) and click **Build**; wait for build completion

8. Navigate to [*CFU tool build root directory*](./ComponentFirmwareUpdateStandAloneToolSample) > **x64** > **Release**. Move the executable **FwUpdateCfu.exe** generated at this location up to the [*CFU_Resources*](./) directory

9. Remove or delete the *ComponentFirmwareUpdateStandAloneToolSample* folder to exclude tool source files from application build


## Performing the update

Use the CFU Standalone Tool executable **FwUpdateCfu.exe** with the following command to execute CFU on the device:

> **Note:** It is suggested to run this step from the Windows Command Prompt. Ensure inclusion of additional arguments (such as `forceIgnoreVersion`), if you plan to utilize specific CFU features with the application.

```sh
FwUpdateCfu.exe update protocolCfgExample_fx2g3.cfg <bitfile_name>.offer.bin <bitfile_name>.payload.bin
```
