When opening the vs2019 example Project, Visual Studio will pop up a dialog asking to retarget Projects to the latest Windows SDK Version and Platform Toolset. DO NOT update the projects because it will break building the libvncxx library. The latest Platform Toolset that can be used to build the libvncxx library is Visual Studio 2017.

Also, the library currently only supports x86 builds.

If you do not have the Visual Studio 2017 Build Tools, you will get the following error:

Error	MSB8020	The build tools for Visual Studio 2017 (Platform Toolset = 'v141') cannot be found. To build using the v141 build tools, please install Visual Studio 2017 build tools.  Alternatively, you may upgrade to the current Visual Studio tools by selecting the Project menu or right-click the solution, and then selecting "Retarget solution".	

The following link will walk you through how to add the Visual Studio 2017 platform toolset to your environment.

https://docs.microsoft.com/en-us/visualstudio/install/modify-visual-studio?view=vs-2019

Run C:\Program Files (x86)\Microsoft Visual Studio\Installer\vs_installer.exe

Select C++/CLI support for v141 build tools (14.16) under Compilers, build tools, and runtimes from the Individual components tab of modifying Visual Studio 2019 

MSVC v141 - VS 2017 C++ x64/x86 build tools (v14.16)

Also select Windows 10 SDK 10.0.18362.0 for the needed make tools.

After adding support for Visual Studio 2017 build toolset, if the projects are marked as unusable, have Visual Studio install the missing components.



