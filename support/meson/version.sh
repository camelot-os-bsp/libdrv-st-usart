#!/usr/bin/env sh

# SPDX-FileCopyrightText: 2023 - 2024 Ledger SAS
# SPDX-License-Identifier: Apache-2.0

if [ "$1" = "get-vcs" ]; then
    git -C "$MESON_SOURCE_ROOT" describe --tags --dirty --always
elif [ "$1" = "set-dist" ]; then
    $MESONREWRITE --sourcedir="$MESON_PROJECT_DIST_ROOT" kwargs set project / version "$2"
else
    exit 1
fi
