// Copyright (c) 2026 LazyDevelopers <lazydeveloper24@gmail.com>. All rights reserved.
// This plugin is distributed under the Fab Standard License.
//
// This product was independently developed by us while participating in the Epic Project, a developer-support
// program of the KRAFTON JUNGLE GameTech Lab. All rights, title, and interest in and to the product are exclusively
// vested in us. Krafton, Inc. was not involved in its development and distribution and disclaims all representations
// and warranties, express or implied, and assumes no responsibility or liability for any consequences arising from
// the use of this product.


#include "Debug/DebugConsoleVariables.h"

int32 Simplify_Toggle = 1;

static FAutoConsoleVariableRef CVar_Simplify(
	TEXT("RDM.Enable.Simplify"),
	Simplify_Toggle,
	TEXT("0=off, 1=on"),
	ECVF_Cheat);