# "Control Software for Nixie Clock" Bachelor Thesis

This repository contains the source files from the "Control Software for Nixie Clock" bachelor thesis.

The repository contains some sources that fall under ©️ STMicroelectronics. Copyright provided where due. All original work licensed under MIT.

## Thesis Abstract

The aim of this thesis is to create control software for a custom-made embedded system using an STM32WB microcontroller unit that can be configured via Bluetooth Low Energy (BLE). The thesis concerns itself with analyzing the criteria placed on the software and the development of said software. The thesis also places emphasis on compatibility with the complementary BLE Android application that was developed for the custom device, but it does not cover the development of the application itself. The software is made from the ground up, because no suitable existing project was found that could be used as its basis. The result of the thesis is functional control software that can be used on the custom-made devices or as a basis for a similar project.

## Thesis Keywords

STM32WB, embedded system, control software, Bluetooth Low Energy, Generic Attribute Profile, Generic Access Profile, real-time clock

## Structure

The repository is split into 4 branches:

- master
- legacy
- wb55cgux
- wb35ceuxa

### master

The master branch holds the template folder, which has the following structure:

- template
  - Core
    - Inc
    - Src
  - STM32_WPAN
    - App

The template folder holds helper files that should be copied into new STM32CubeIDE projects. All the folders belong in the IDE subfolders of the same name.

### legacy

This branch holds the original STM32CubeIDE project that was used during development.

### wb55cgux

This branch holds a brand-new, cleaned-up STM32CubeIDE project for STM32WB55CGUX MCUs.

### wb35ceuxa

This branch holds a brand-new, cleaned-up STM32CubeIDE project for STM32WB35CEUXA MCUs.
