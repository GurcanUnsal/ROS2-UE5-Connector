# Makefile generated by MakefileGenerator.cs
# *DO NOT EDIT*

UNREALROOTPATH = /home/gurcan/Desktop/Linux_Unreal_Engine_5.3.0
GAMEPROJECTFILE =/home/gurcan/Documents/Unreal Projects/ROS_Controller/ROS_Controller.uproject

TARGETS = \
	UnrealEditor-Linux-DebugGame  \
	UnrealEditor-Linux-Shipping  \
	UnrealEditor \
	UnrealGame-Linux-DebugGame  \
	UnrealGame-Linux-Shipping  \
	UnrealGame \
	ROS_Controller-Linux-DebugGame  \
	ROS_Controller-Linux-Shipping  \
	ROS_Controller \
	ROS_ControllerEditor-Linux-DebugGame  \
	ROS_ControllerEditor-Linux-Shipping  \
	ROS_ControllerEditor\
	configure

BUILD = bash "$(UNREALROOTPATH)/Engine/Build/BatchFiles/Linux/Build.sh"
PROJECTBUILD = "$(UNREALROOTPATH)/Engine/Binaries/ThirdParty/DotNet/6.0.302/linux/dotnet" "$(UNREALROOTPATH)/Engine/Binaries/DotNET/UnrealBuildTool/UnrealBuildTool.dll"

all: StandardSet

RequiredTools: CrashReportClient-Linux-Shipping CrashReportClientEditor-Linux-Shipping ShaderCompileWorker UnrealLightmass EpicWebHelper-Linux-Shipping

StandardSet: RequiredTools UnrealFrontend ROS_ControllerEditor UnrealInsights

DebugSet: RequiredTools UnrealFrontend-Linux-Debug ROS_ControllerEditor-Linux-Debug


UnrealEditor-Linux-DebugGame:
	 $(BUILD) UnrealEditor Linux DebugGame  $(ARGS)

UnrealEditor-Linux-Shipping:
	 $(BUILD) UnrealEditor Linux Shipping  $(ARGS)

UnrealEditor:
	 $(BUILD) UnrealEditor Linux Development  $(ARGS)

UnrealGame-Linux-DebugGame:
	 $(BUILD) UnrealGame Linux DebugGame  $(ARGS)

UnrealGame-Linux-Shipping:
	 $(BUILD) UnrealGame Linux Shipping  $(ARGS)

UnrealGame:
	 $(BUILD) UnrealGame Linux Development  $(ARGS)

ROS_Controller-Linux-DebugGame:
	 $(PROJECTBUILD) ROS_Controller Linux DebugGame  -project="$(GAMEPROJECTFILE)" $(ARGS)

ROS_Controller-Linux-Shipping:
	 $(PROJECTBUILD) ROS_Controller Linux Shipping  -project="$(GAMEPROJECTFILE)" $(ARGS)

ROS_Controller:
	 $(PROJECTBUILD) ROS_Controller Linux Development  -project="$(GAMEPROJECTFILE)" $(ARGS)

ROS_ControllerEditor-Linux-DebugGame:
	 $(PROJECTBUILD) ROS_ControllerEditor Linux DebugGame  -project="$(GAMEPROJECTFILE)" $(ARGS)

ROS_ControllerEditor-Linux-Shipping:
	 $(PROJECTBUILD) ROS_ControllerEditor Linux Shipping  -project="$(GAMEPROJECTFILE)" $(ARGS)

ROS_ControllerEditor:
	 $(PROJECTBUILD) ROS_ControllerEditor Linux Development  -project="$(GAMEPROJECTFILE)" $(ARGS)

configure:
	xbuild /property:Configuration=Development /verbosity:quiet /nologo "$(UNREALROOTPATH)/Engine/Source/Programs/UnrealBuildTool/UnrealBuildTool.csproj"
	$(PROJECTBUILD) -projectfiles -project="\"$(GAMEPROJECTFILE)\"" -game -engine 

.PHONY: $(TARGETS)
