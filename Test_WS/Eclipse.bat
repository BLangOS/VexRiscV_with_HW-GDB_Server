@echo off

rem Pfad auf "Windows Build Tools" bekannt machen
PATH L:\tools\xpack-windows-build-tools-4.2.1-2\bin;%PATH%
rem Pfad zum Eclipse executable und zum gewuenschten Workspace setzen
set ECLIPSE_PATH=L:\tools\eclipse-cpp-2020-09-R-win32-x86_64\eclipse
set WORKSPACE=.\
%ECLIPSE_PATH%\eclipse.exe -data %WORKSPACE%
