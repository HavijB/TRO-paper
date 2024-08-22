function [] = Import_to_Simulink()
%% Create code importer object
obj = Simulink.CodeImporter;

%% Set library file name and output folder
obj.LibraryFileName = "g_Matrix";
obj.OutputFolder = ".";

%% Set the custom code to import
obj.CustomCode.InterfaceHeaders = "g_Matrix.h";
obj.CustomCode.IncludePaths = "./include";
obj.CustomCode.SourceFiles = "g_Matrix.m";

%% Parse custom code and configure function
obj.parse();
fcn = obj.ParseInfo.getFunctions("g_Matrix");
fcn.PortSpecification.ReturnArgument.Label = "g_Matrix";

%% Import function "Controller"
obj.import('functions', ["g_Matrix"]);
end