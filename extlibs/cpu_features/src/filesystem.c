// Copyright 2017 Google Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//    http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "internal/filesystem.h"

#include <errno.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>

#if defined(_MSC_VER)
#include <io.h>
int OpenFile(const char* filename) { return _open(filename, _O_RDONLY); }

void CloseFile(int file_descriptor) { _close(file_descriptor); }

int ReadFile(int file_descriptor, void* buffer, size_t buffer_size) {
  return (int)_read(file_descriptor, buffer, (unsigned int)buffer_size);
}

#else
#include <unistd.h>

int OpenFile(const char* filename) {
  int result;
  do {
    result = open(filename, O_RDONLY);
  } while (result == -1L && errno == EINTR);
  return result;
}

void CloseFile(int file_descriptor) { close(file_descriptor); }

int ReadFile(int file_descriptor, void* buffer, size_t buffer_size) {
  int result;
  do {
    result = read(file_descriptor, buffer, buffer_size);
  } while (result == -1L && errno == EINTR);
  return result;
}

#endif
