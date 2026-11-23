#include "example_archive.hpp"

#include <archive.h>
#include <archive_entry.h>

#include <array>
#include <fstream>
#include <map>
#include <memory>
#include <set>
#include <stdexcept>
#include <string_view>

namespace lidar_viewer {
namespace fs=std::filesystem;
namespace {

bool PortableBasename(std::string_view name) {
  constexpr std::string_view allowed="ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789-_.";
  return !name.empty() && name.front()!='.' && name.back()!='.' && name.find_first_not_of(allowed)==name.npos;
}

std::string FoldAscii(std::string_view name) {
  std::string folded;
  folded.reserve(name.size());
  for(char character:name)
    folded.push_back(character>='A' && character<='Z' ? static_cast<char>(character-'A'+'a') : character);
  return folded;
}

bool IsDeviceName(std::string_view folded) {
  const auto stem=folded.substr(0,folded.find('.'));
  return stem=="con" || stem=="prn" || stem=="aux" || stem=="nul" ||
    (stem.size()==4 && (stem.substr(0,3)=="com" || stem.substr(0,3)=="lpt") &&
      stem.back()>='0' && stem.back()<='9');
}

[[noreturn]] void ThrowArchiveError(struct archive * reader, const char * message) {
  const auto error=archive_error_string(reader);
  throw std::runtime_error(error ? std::string(message)+": "+error : message);
}

void RequireArchiveSuccess(struct archive * reader, int status, const char * message) {
  if(status!=ARCHIVE_OK) ThrowArchiveError(reader,message);
}

void CheckCanceled(const std::function<bool()> & keep_running) {
  if(!keep_running()) throw std::runtime_error("Download canceled");
}

void UnpackMember(struct archive * reader, const fs::path & path, const ExampleFile & member,
    const std::function<bool()> & keep_running) {
  std::ofstream output(path,std::ios::binary);
  if(!output) throw std::runtime_error("Cannot create example archive member "+path.filename().u8string());
  std::array<char,65536> buffer{};
  std::uint64_t remaining=member.bytes;
  for(;;) {
    CheckCanceled(keep_running);
    const auto bytes=archive_read_data(reader,buffer.data(),buffer.size());
    if(bytes==0) break;
    if(bytes<0) ThrowArchiveError(reader,"Cannot read example archive member");
    if(static_cast<std::uint64_t>(bytes)>remaining) throw std::runtime_error("Example archive member exceeds its pinned size");
    output.write(buffer.data(),static_cast<std::streamsize>(bytes));
    if(!output) throw std::runtime_error("Cannot write example archive member");
    remaining-=static_cast<std::uint64_t>(bytes);
  }
  output.close();
  if(!output || remaining) throw std::runtime_error("Incomplete example archive member");
}

} // namespace

void UnpackExampleArchive(const fs::path & zip, const fs::path & stage,
    const ExampleFile * members, std::size_t count, const std::function<bool()> & keep_running) {
  CheckCanceled(keep_running);
  if(!fs::is_directory(fs::symlink_status(stage)) || !fs::is_empty(stage))
    throw std::runtime_error("Example archive staging directory is unavailable");
  if(!members || count==0) throw std::runtime_error("Missing example archive manifest");

  std::map<std::string_view,const ExampleFile *> expected;
  std::set<std::string> outputs;
  for(std::size_t index=0;index<count;++index) {
    const auto & member=members[index];
    if(!member.name) throw std::runtime_error("Missing example archive member name");
    const std::string_view source=member.archive_name ? member.archive_name : member.name;
    const auto folded=FoldAscii(member.name);
    if(!PortableBasename(source) || !PortableBasename(member.name) || IsDeviceName(folded) ||
       !expected.emplace(source,&member).second || !outputs.insert(folded).second)
      throw std::runtime_error("Invalid or duplicate example archive member name");
  }

  std::unique_ptr<struct archive,decltype(&archive_read_free)> reader(archive_read_new(),archive_read_free);
  if(!reader) throw std::runtime_error("Cannot initialize example archive reader");
  RequireArchiveSuccess(reader.get(),archive_read_support_format_zip_seekable(reader.get()),"Cannot enable ZIP support");
#ifdef _WIN32
  const auto opened=archive_read_open_filename_w(reader.get(),zip.c_str(),65536);
#else
  const auto opened=archive_read_open_filename(reader.get(),zip.c_str(),65536);
#endif
  RequireArchiveSuccess(reader.get(),opened,"Cannot open example archive");

  for(;;) {
    CheckCanceled(keep_running);
    struct archive_entry * entry=nullptr;
    const auto status=archive_read_next_header(reader.get(),&entry);
    if(status==ARCHIVE_EOF) break;
    RequireArchiveSuccess(reader.get(),status,"Cannot read example archive header");
    const auto name=archive_entry_pathname_utf8(entry);
    const auto found=name ? expected.find(name) : expected.end();
    if(found==expected.end() || archive_entry_filetype(entry)!=AE_IFREG ||
       archive_entry_symlink(entry) || archive_entry_hardlink(entry) || archive_entry_is_encrypted(entry)>0)
      throw std::runtime_error("Unexpected or duplicate member in example archive");

    const auto & member=*found->second;
    const auto size=archive_entry_size(entry);
    if(!archive_entry_size_is_set(entry) || size<0 || static_cast<std::uint64_t>(size)!=member.bytes)
      throw std::runtime_error("Example archive member size differs from its manifest");
    UnpackMember(reader.get(),stage/fs::u8path(member.name),member,keep_running);
    expected.erase(found);
  }
  if(!expected.empty()) throw std::runtime_error("Example archive is missing required members");
  RequireArchiveSuccess(reader.get(),archive_read_close(reader.get()),"Cannot finish reading example archive");
}

} // namespace lidar_viewer
