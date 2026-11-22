#include "model_process.hpp"

#include <algorithm>
#include <stdexcept>
#include <string>
#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#else
#include <cerrno>
#include <csignal>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>
extern char ** environ;
#endif

namespace lidar_viewer::inference
{
ModelProcess::~ModelProcess()
{
  Close();
}

void ModelProcess::Open(const std::filesystem::path & python,
  const std::vector<std::filesystem::path> & arguments, const std::filesystem::path & log)
{
  log_path_ = log;
#ifdef _WIN32
  SECURITY_ATTRIBUTES attributes{sizeof(SECURITY_ATTRIBUTES), nullptr, TRUE};
  HANDLE child_in = nullptr, child_out = nullptr;
  if (!CreatePipe(&child_in, &write_, &attributes, 0)) throw std::runtime_error("Cannot create detector pipes");
  if (!CreatePipe(&read_, &child_out, &attributes, 0)) {
    CloseHandle(child_in);
    throw std::runtime_error("Cannot create detector pipes");
  }
  SetHandleInformation(write_, HANDLE_FLAG_INHERIT, 0);
  SetHandleInformation(read_, HANDLE_FLAG_INHERIT, 0);
  HANDLE error = CreateFileW(log.c_str(), GENERIC_WRITE, FILE_SHARE_READ | FILE_SHARE_WRITE,
    &attributes, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
  auto quote = [](const std::wstring & value) {
    std::wstring result = L"\"";
    std::size_t slashes = 0;
    for (wchar_t c : value) {
      if (c == L'\\') {
        ++slashes;
        continue;
      }
      result.append(c == L'"' ? slashes * 2 + 1 : slashes, L'\\');
      slashes = 0;
      result += c;
    }
    result.append(slashes * 2, L'\\');
    return result + L'"';
  };
  std::wstring command = quote(python.wstring());
  for (const auto & argument : arguments) command += L" " + quote(argument.wstring());
  STARTUPINFOW start{};
  start.cb = sizeof(start);
  start.dwFlags = STARTF_USESTDHANDLES;
  start.hStdInput = child_in;
  start.hStdOutput = child_out;
  start.hStdError = error;
  PROCESS_INFORMATION process{};
  const bool ok = CreateProcessW(python.c_str(), command.data(), nullptr, nullptr, TRUE,
    CREATE_NO_WINDOW, nullptr, nullptr, &start, &process) != 0;
  CloseHandle(child_in);
  CloseHandle(child_out);
  if (error != INVALID_HANDLE_VALUE) CloseHandle(error);
  if (!ok) throw std::runtime_error("Cannot start detector Python runtime");
  process_ = process.hProcess;
  CloseHandle(process.hThread);
#else
  int input[2], output[2];
  const auto open_pipe = [](int descriptors[2]) {
#ifdef __linux__
    return pipe2(descriptors, O_CLOEXEC);
#else
    const int result = pipe(descriptors);
    if (result == 0) {
      for (int i = 0; i < 2; ++i) fcntl(descriptors[i], F_SETFD, FD_CLOEXEC);
    }
    return result;
#endif
  };
  if (open_pipe(input) != 0) throw std::runtime_error("Cannot create model input pipe");
  if (open_pipe(output) != 0) {
    close(input[0]);
    close(input[1]);
    throw std::runtime_error("Cannot create model output pipe");
  }
  const int error = open(log.c_str(), O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0600);
  posix_spawn_file_actions_t actions;
  posix_spawn_file_actions_init(&actions);
  posix_spawn_file_actions_adddup2(&actions, input[0], STDIN_FILENO);
  posix_spawn_file_actions_adddup2(&actions, output[1], STDOUT_FILENO);
  if (error >= 0) posix_spawn_file_actions_adddup2(&actions, error, STDERR_FILENO);
  std::vector<std::string> args{python.string()};
  for (const auto & argument : arguments) args.push_back(argument.string());
  std::vector<char *> argv;
  for (auto & arg : args) argv.push_back(arg.data());
  argv.push_back(nullptr);
  const int rc = posix_spawnp(&pid_, python.c_str(), &actions, nullptr, argv.data(), environ);
  posix_spawn_file_actions_destroy(&actions);
  close(input[0]);
  close(output[1]);
  if (error >= 0) close(error);
  if (rc != 0) {
    pid_ = -1;
    close(input[1]);
    close(output[0]);
    throw std::runtime_error(std::string("Cannot start detector Python runtime: ") + std::strerror(rc));
  }
  write_ = input[1];
  read_ = output[0];
#endif
}

void ModelProcess::Kill()
{
#ifdef _WIN32
  if (process_) TerminateProcess(process_, 1);
#else
  if (pid_ > 0) kill(pid_, SIGKILL);
#endif
}

void ModelProcess::Close()
{
  Kill();
#ifdef _WIN32
  if (process_) {
    WaitForSingleObject(process_, 5000);
    CloseHandle(process_);
    process_ = nullptr;
  }
  if (read_) {
    CloseHandle(read_);
    read_ = nullptr;
  }
  if (write_) {
    CloseHandle(write_);
    write_ = nullptr;
  }
#else
  if (pid_ > 0) {
    while (waitpid(pid_, nullptr, 0) < 0 && errno == EINTR) {}
    pid_ = -1;
  }
  if (read_ >= 0) {
    close(read_);
    read_ = -1;
  }
  if (write_ >= 0) {
    close(write_);
    write_ = -1;
  }
#endif
}

void ModelProcess::Read(void * data, std::size_t size)
{
  Transfer(data, size, false);
}

void ModelProcess::Write(const void * data, std::size_t size)
{
  Transfer(const_cast<void *>(data), size, true);
}

#ifndef _WIN32
void ModelProcess::SetDeadline(std::chrono::seconds timeout)
{
  deadline_ = std::chrono::steady_clock::now() + timeout;
}
#endif

void ModelProcess::Transfer(void * data, std::size_t size, bool writing)
{
  auto * bytes = static_cast<unsigned char *>(data);
  while (size) {
#ifdef _WIN32
    DWORD transferred = 0;
    const auto count = static_cast<DWORD>(std::min<std::size_t>(size, 1 << 20));
    const bool ok = writing ? WriteFile(write_, bytes, count, &transferred, nullptr) != 0 :
      ReadFile(read_, bytes, count, &transferred, nullptr) != 0;
    if (!ok || !transferred) throw std::runtime_error("Model process disconnected; see " + log_path_.filename().u8string());
#else
    std::size_t count = size;
    if (deadline_) {
      const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
        *deadline_ - std::chrono::steady_clock::now());
      if (remaining.count() <= 0) throw std::runtime_error("Model process timed out");
      pollfd descriptor{writing ? write_ : read_, static_cast<short>(writing ? POLLOUT : POLLIN), 0};
      const int ready = poll(&descriptor, 1, static_cast<int>(remaining.count()));
      if (ready < 0 && errno == EINTR) continue;
      if (!ready) throw std::runtime_error("Model process timed out");
      if (ready < 0) throw std::runtime_error("Cannot poll model process");
      if (writing) count = std::min<std::size_t>(count, 4096);
    }
    const auto transferred = writing ? write(write_, bytes, count) : read(read_, bytes, count);
    if (transferred < 0 && errno == EINTR) continue;
    if (transferred <= 0) throw std::runtime_error("Model process disconnected; see " + log_path_.filename().u8string());
#endif
    bytes += transferred;
    size -= transferred;
  }
}
} // namespace lidar_viewer::inference
