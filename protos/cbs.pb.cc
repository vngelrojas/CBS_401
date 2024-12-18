// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: cbs.proto

#include "cbs.pb.h"

#include <algorithm>

#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/extension_set.h>
#include <google/protobuf/wire_format_lite.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/reflection_ops.h>
#include <google/protobuf/wire_format.h>
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>

PROTOBUF_PRAGMA_INIT_SEG

namespace _pb = ::PROTOBUF_NAMESPACE_ID;
namespace _pbi = _pb::internal;

namespace CBSProto {
PROTOBUF_CONSTEXPR CBS_Location::CBS_Location(
    ::_pbi::ConstantInitialized): _impl_{
    /*decltype(_impl_.x_)*/0
  , /*decltype(_impl_.y_)*/0
  , /*decltype(_impl_._cached_size_)*/{}} {}
struct CBS_LocationDefaultTypeInternal {
  PROTOBUF_CONSTEXPR CBS_LocationDefaultTypeInternal()
      : _instance(::_pbi::ConstantInitialized{}) {}
  ~CBS_LocationDefaultTypeInternal() {}
  union {
    CBS_Location _instance;
  };
};
PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT PROTOBUF_ATTRIBUTE_INIT_PRIORITY1 CBS_LocationDefaultTypeInternal _CBS_Location_default_instance_;
PROTOBUF_CONSTEXPR CBS_State::CBS_State(
    ::_pbi::ConstantInitialized): _impl_{
    /*decltype(_impl_.time_)*/0
  , /*decltype(_impl_.x_)*/0
  , /*decltype(_impl_.y_)*/0
  , /*decltype(_impl_._cached_size_)*/{}} {}
struct CBS_StateDefaultTypeInternal {
  PROTOBUF_CONSTEXPR CBS_StateDefaultTypeInternal()
      : _instance(::_pbi::ConstantInitialized{}) {}
  ~CBS_StateDefaultTypeInternal() {}
  union {
    CBS_State _instance;
  };
};
PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT PROTOBUF_ATTRIBUTE_INIT_PRIORITY1 CBS_StateDefaultTypeInternal _CBS_State_default_instance_;
PROTOBUF_CONSTEXPR CBS::CBS(
    ::_pbi::ConstantInitialized): _impl_{
    /*decltype(_impl_.obstacles_)*/{}
  , /*decltype(_impl_.goals_)*/{}
  , /*decltype(_impl_.start_states_)*/{}
  , /*decltype(_impl_.num_of_rows_)*/0
  , /*decltype(_impl_.num_of_cols_)*/0
  , /*decltype(_impl_.world_size_)*/0
  , /*decltype(_impl_.world_rank_)*/0
  , /*decltype(_impl_._cached_size_)*/{}} {}
struct CBSDefaultTypeInternal {
  PROTOBUF_CONSTEXPR CBSDefaultTypeInternal()
      : _instance(::_pbi::ConstantInitialized{}) {}
  ~CBSDefaultTypeInternal() {}
  union {
    CBS _instance;
  };
};
PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT PROTOBUF_ATTRIBUTE_INIT_PRIORITY1 CBSDefaultTypeInternal _CBS_default_instance_;
}  // namespace CBSProto
static ::_pb::Metadata file_level_metadata_cbs_2eproto[3];
static constexpr ::_pb::EnumDescriptor const** file_level_enum_descriptors_cbs_2eproto = nullptr;
static constexpr ::_pb::ServiceDescriptor const** file_level_service_descriptors_cbs_2eproto = nullptr;

const uint32_t TableStruct_cbs_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  ~0u,  // no _has_bits_
  PROTOBUF_FIELD_OFFSET(::CBSProto::CBS_Location, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  ~0u,  // no _inlined_string_donated_
  PROTOBUF_FIELD_OFFSET(::CBSProto::CBS_Location, _impl_.x_),
  PROTOBUF_FIELD_OFFSET(::CBSProto::CBS_Location, _impl_.y_),
  ~0u,  // no _has_bits_
  PROTOBUF_FIELD_OFFSET(::CBSProto::CBS_State, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  ~0u,  // no _inlined_string_donated_
  PROTOBUF_FIELD_OFFSET(::CBSProto::CBS_State, _impl_.time_),
  PROTOBUF_FIELD_OFFSET(::CBSProto::CBS_State, _impl_.x_),
  PROTOBUF_FIELD_OFFSET(::CBSProto::CBS_State, _impl_.y_),
  ~0u,  // no _has_bits_
  PROTOBUF_FIELD_OFFSET(::CBSProto::CBS, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  ~0u,  // no _inlined_string_donated_
  PROTOBUF_FIELD_OFFSET(::CBSProto::CBS, _impl_.num_of_rows_),
  PROTOBUF_FIELD_OFFSET(::CBSProto::CBS, _impl_.num_of_cols_),
  PROTOBUF_FIELD_OFFSET(::CBSProto::CBS, _impl_.world_size_),
  PROTOBUF_FIELD_OFFSET(::CBSProto::CBS, _impl_.world_rank_),
  PROTOBUF_FIELD_OFFSET(::CBSProto::CBS, _impl_.obstacles_),
  PROTOBUF_FIELD_OFFSET(::CBSProto::CBS, _impl_.goals_),
  PROTOBUF_FIELD_OFFSET(::CBSProto::CBS, _impl_.start_states_),
};
static const ::_pbi::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, -1, -1, sizeof(::CBSProto::CBS_Location)},
  { 8, -1, -1, sizeof(::CBSProto::CBS_State)},
  { 17, -1, -1, sizeof(::CBSProto::CBS)},
};

static const ::_pb::Message* const file_default_instances[] = {
  &::CBSProto::_CBS_Location_default_instance_._instance,
  &::CBSProto::_CBS_State_default_instance_._instance,
  &::CBSProto::_CBS_default_instance_._instance,
};

const char descriptor_table_protodef_cbs_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n\tcbs.proto\022\010CBSProto\"\243\002\n\003CBS\022\023\n\013num_of_"
  "rows\030\001 \001(\005\022\023\n\013num_of_cols\030\002 \001(\005\022\022\n\nworld"
  "_size\030\003 \001(\005\022\022\n\nworld_rank\030\004 \001(\005\022)\n\tobsta"
  "cles\030\005 \003(\0132\026.CBSProto.CBS.Location\022%\n\005go"
  "als\030\006 \003(\0132\026.CBSProto.CBS.Location\022)\n\014sta"
  "rt_states\030\007 \003(\0132\023.CBSProto.CBS.State\032 \n\010"
  "Location\022\t\n\001x\030\001 \001(\005\022\t\n\001y\030\002 \001(\005\032+\n\005State\022"
  "\014\n\004time\030\001 \001(\005\022\t\n\001x\030\002 \001(\005\022\t\n\001y\030\003 \001(\005b\006pro"
  "to3"
  ;
static ::_pbi::once_flag descriptor_table_cbs_2eproto_once;
const ::_pbi::DescriptorTable descriptor_table_cbs_2eproto = {
    false, false, 323, descriptor_table_protodef_cbs_2eproto,
    "cbs.proto",
    &descriptor_table_cbs_2eproto_once, nullptr, 0, 3,
    schemas, file_default_instances, TableStruct_cbs_2eproto::offsets,
    file_level_metadata_cbs_2eproto, file_level_enum_descriptors_cbs_2eproto,
    file_level_service_descriptors_cbs_2eproto,
};
PROTOBUF_ATTRIBUTE_WEAK const ::_pbi::DescriptorTable* descriptor_table_cbs_2eproto_getter() {
  return &descriptor_table_cbs_2eproto;
}

// Force running AddDescriptors() at dynamic initialization time.
PROTOBUF_ATTRIBUTE_INIT_PRIORITY2 static ::_pbi::AddDescriptorsRunner dynamic_init_dummy_cbs_2eproto(&descriptor_table_cbs_2eproto);
namespace CBSProto {

// ===================================================================

class CBS_Location::_Internal {
 public:
};

CBS_Location::CBS_Location(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                         bool is_message_owned)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena, is_message_owned) {
  SharedCtor(arena, is_message_owned);
  // @@protoc_insertion_point(arena_constructor:CBSProto.CBS.Location)
}
CBS_Location::CBS_Location(const CBS_Location& from)
  : ::PROTOBUF_NAMESPACE_ID::Message() {
  CBS_Location* const _this = this; (void)_this;
  new (&_impl_) Impl_{
      decltype(_impl_.x_){}
    , decltype(_impl_.y_){}
    , /*decltype(_impl_._cached_size_)*/{}};

  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  ::memcpy(&_impl_.x_, &from._impl_.x_,
    static_cast<size_t>(reinterpret_cast<char*>(&_impl_.y_) -
    reinterpret_cast<char*>(&_impl_.x_)) + sizeof(_impl_.y_));
  // @@protoc_insertion_point(copy_constructor:CBSProto.CBS.Location)
}

inline void CBS_Location::SharedCtor(
    ::_pb::Arena* arena, bool is_message_owned) {
  (void)arena;
  (void)is_message_owned;
  new (&_impl_) Impl_{
      decltype(_impl_.x_){0}
    , decltype(_impl_.y_){0}
    , /*decltype(_impl_._cached_size_)*/{}
  };
}

CBS_Location::~CBS_Location() {
  // @@protoc_insertion_point(destructor:CBSProto.CBS.Location)
  if (auto *arena = _internal_metadata_.DeleteReturnArena<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>()) {
  (void)arena;
    return;
  }
  SharedDtor();
}

inline void CBS_Location::SharedDtor() {
  GOOGLE_DCHECK(GetArenaForAllocation() == nullptr);
}

void CBS_Location::SetCachedSize(int size) const {
  _impl_._cached_size_.Set(size);
}

void CBS_Location::Clear() {
// @@protoc_insertion_point(message_clear_start:CBSProto.CBS.Location)
  uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  ::memset(&_impl_.x_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&_impl_.y_) -
      reinterpret_cast<char*>(&_impl_.x_)) + sizeof(_impl_.y_));
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* CBS_Location::_InternalParse(const char* ptr, ::_pbi::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  while (!ctx->Done(&ptr)) {
    uint32_t tag;
    ptr = ::_pbi::ReadTag(ptr, &tag);
    switch (tag >> 3) {
      // int32 x = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 8)) {
          _impl_.x_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint32(&ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // int32 y = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 16)) {
          _impl_.y_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint32(&ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      default:
        goto handle_unusual;
    }  // switch
  handle_unusual:
    if ((tag == 0) || ((tag & 7) == 4)) {
      CHK_(ptr);
      ctx->SetLastTag(tag);
      goto message_done;
    }
    ptr = UnknownFieldParse(
        tag,
        _internal_metadata_.mutable_unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(),
        ptr, ctx);
    CHK_(ptr != nullptr);
  }  // while
message_done:
  return ptr;
failure:
  ptr = nullptr;
  goto message_done;
#undef CHK_
}

uint8_t* CBS_Location::_InternalSerialize(
    uint8_t* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:CBSProto.CBS.Location)
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  // int32 x = 1;
  if (this->_internal_x() != 0) {
    target = stream->EnsureSpace(target);
    target = ::_pbi::WireFormatLite::WriteInt32ToArray(1, this->_internal_x(), target);
  }

  // int32 y = 2;
  if (this->_internal_y() != 0) {
    target = stream->EnsureSpace(target);
    target = ::_pbi::WireFormatLite::WriteInt32ToArray(2, this->_internal_y(), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::_pbi::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:CBSProto.CBS.Location)
  return target;
}

size_t CBS_Location::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:CBSProto.CBS.Location)
  size_t total_size = 0;

  uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // int32 x = 1;
  if (this->_internal_x() != 0) {
    total_size += ::_pbi::WireFormatLite::Int32SizePlusOne(this->_internal_x());
  }

  // int32 y = 2;
  if (this->_internal_y() != 0) {
    total_size += ::_pbi::WireFormatLite::Int32SizePlusOne(this->_internal_y());
  }

  return MaybeComputeUnknownFieldsSize(total_size, &_impl_._cached_size_);
}

const ::PROTOBUF_NAMESPACE_ID::Message::ClassData CBS_Location::_class_data_ = {
    ::PROTOBUF_NAMESPACE_ID::Message::CopyWithSourceCheck,
    CBS_Location::MergeImpl
};
const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*CBS_Location::GetClassData() const { return &_class_data_; }


void CBS_Location::MergeImpl(::PROTOBUF_NAMESPACE_ID::Message& to_msg, const ::PROTOBUF_NAMESPACE_ID::Message& from_msg) {
  auto* const _this = static_cast<CBS_Location*>(&to_msg);
  auto& from = static_cast<const CBS_Location&>(from_msg);
  // @@protoc_insertion_point(class_specific_merge_from_start:CBSProto.CBS.Location)
  GOOGLE_DCHECK_NE(&from, _this);
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  if (from._internal_x() != 0) {
    _this->_internal_set_x(from._internal_x());
  }
  if (from._internal_y() != 0) {
    _this->_internal_set_y(from._internal_y());
  }
  _this->_internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
}

void CBS_Location::CopyFrom(const CBS_Location& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:CBSProto.CBS.Location)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool CBS_Location::IsInitialized() const {
  return true;
}

void CBS_Location::InternalSwap(CBS_Location* other) {
  using std::swap;
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::internal::memswap<
      PROTOBUF_FIELD_OFFSET(CBS_Location, _impl_.y_)
      + sizeof(CBS_Location::_impl_.y_)
      - PROTOBUF_FIELD_OFFSET(CBS_Location, _impl_.x_)>(
          reinterpret_cast<char*>(&_impl_.x_),
          reinterpret_cast<char*>(&other->_impl_.x_));
}

::PROTOBUF_NAMESPACE_ID::Metadata CBS_Location::GetMetadata() const {
  return ::_pbi::AssignDescriptors(
      &descriptor_table_cbs_2eproto_getter, &descriptor_table_cbs_2eproto_once,
      file_level_metadata_cbs_2eproto[0]);
}

// ===================================================================

class CBS_State::_Internal {
 public:
};

CBS_State::CBS_State(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                         bool is_message_owned)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena, is_message_owned) {
  SharedCtor(arena, is_message_owned);
  // @@protoc_insertion_point(arena_constructor:CBSProto.CBS.State)
}
CBS_State::CBS_State(const CBS_State& from)
  : ::PROTOBUF_NAMESPACE_ID::Message() {
  CBS_State* const _this = this; (void)_this;
  new (&_impl_) Impl_{
      decltype(_impl_.time_){}
    , decltype(_impl_.x_){}
    , decltype(_impl_.y_){}
    , /*decltype(_impl_._cached_size_)*/{}};

  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  ::memcpy(&_impl_.time_, &from._impl_.time_,
    static_cast<size_t>(reinterpret_cast<char*>(&_impl_.y_) -
    reinterpret_cast<char*>(&_impl_.time_)) + sizeof(_impl_.y_));
  // @@protoc_insertion_point(copy_constructor:CBSProto.CBS.State)
}

inline void CBS_State::SharedCtor(
    ::_pb::Arena* arena, bool is_message_owned) {
  (void)arena;
  (void)is_message_owned;
  new (&_impl_) Impl_{
      decltype(_impl_.time_){0}
    , decltype(_impl_.x_){0}
    , decltype(_impl_.y_){0}
    , /*decltype(_impl_._cached_size_)*/{}
  };
}

CBS_State::~CBS_State() {
  // @@protoc_insertion_point(destructor:CBSProto.CBS.State)
  if (auto *arena = _internal_metadata_.DeleteReturnArena<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>()) {
  (void)arena;
    return;
  }
  SharedDtor();
}

inline void CBS_State::SharedDtor() {
  GOOGLE_DCHECK(GetArenaForAllocation() == nullptr);
}

void CBS_State::SetCachedSize(int size) const {
  _impl_._cached_size_.Set(size);
}

void CBS_State::Clear() {
// @@protoc_insertion_point(message_clear_start:CBSProto.CBS.State)
  uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  ::memset(&_impl_.time_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&_impl_.y_) -
      reinterpret_cast<char*>(&_impl_.time_)) + sizeof(_impl_.y_));
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* CBS_State::_InternalParse(const char* ptr, ::_pbi::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  while (!ctx->Done(&ptr)) {
    uint32_t tag;
    ptr = ::_pbi::ReadTag(ptr, &tag);
    switch (tag >> 3) {
      // int32 time = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 8)) {
          _impl_.time_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint32(&ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // int32 x = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 16)) {
          _impl_.x_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint32(&ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // int32 y = 3;
      case 3:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 24)) {
          _impl_.y_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint32(&ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      default:
        goto handle_unusual;
    }  // switch
  handle_unusual:
    if ((tag == 0) || ((tag & 7) == 4)) {
      CHK_(ptr);
      ctx->SetLastTag(tag);
      goto message_done;
    }
    ptr = UnknownFieldParse(
        tag,
        _internal_metadata_.mutable_unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(),
        ptr, ctx);
    CHK_(ptr != nullptr);
  }  // while
message_done:
  return ptr;
failure:
  ptr = nullptr;
  goto message_done;
#undef CHK_
}

uint8_t* CBS_State::_InternalSerialize(
    uint8_t* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:CBSProto.CBS.State)
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  // int32 time = 1;
  if (this->_internal_time() != 0) {
    target = stream->EnsureSpace(target);
    target = ::_pbi::WireFormatLite::WriteInt32ToArray(1, this->_internal_time(), target);
  }

  // int32 x = 2;
  if (this->_internal_x() != 0) {
    target = stream->EnsureSpace(target);
    target = ::_pbi::WireFormatLite::WriteInt32ToArray(2, this->_internal_x(), target);
  }

  // int32 y = 3;
  if (this->_internal_y() != 0) {
    target = stream->EnsureSpace(target);
    target = ::_pbi::WireFormatLite::WriteInt32ToArray(3, this->_internal_y(), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::_pbi::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:CBSProto.CBS.State)
  return target;
}

size_t CBS_State::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:CBSProto.CBS.State)
  size_t total_size = 0;

  uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // int32 time = 1;
  if (this->_internal_time() != 0) {
    total_size += ::_pbi::WireFormatLite::Int32SizePlusOne(this->_internal_time());
  }

  // int32 x = 2;
  if (this->_internal_x() != 0) {
    total_size += ::_pbi::WireFormatLite::Int32SizePlusOne(this->_internal_x());
  }

  // int32 y = 3;
  if (this->_internal_y() != 0) {
    total_size += ::_pbi::WireFormatLite::Int32SizePlusOne(this->_internal_y());
  }

  return MaybeComputeUnknownFieldsSize(total_size, &_impl_._cached_size_);
}

const ::PROTOBUF_NAMESPACE_ID::Message::ClassData CBS_State::_class_data_ = {
    ::PROTOBUF_NAMESPACE_ID::Message::CopyWithSourceCheck,
    CBS_State::MergeImpl
};
const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*CBS_State::GetClassData() const { return &_class_data_; }


void CBS_State::MergeImpl(::PROTOBUF_NAMESPACE_ID::Message& to_msg, const ::PROTOBUF_NAMESPACE_ID::Message& from_msg) {
  auto* const _this = static_cast<CBS_State*>(&to_msg);
  auto& from = static_cast<const CBS_State&>(from_msg);
  // @@protoc_insertion_point(class_specific_merge_from_start:CBSProto.CBS.State)
  GOOGLE_DCHECK_NE(&from, _this);
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  if (from._internal_time() != 0) {
    _this->_internal_set_time(from._internal_time());
  }
  if (from._internal_x() != 0) {
    _this->_internal_set_x(from._internal_x());
  }
  if (from._internal_y() != 0) {
    _this->_internal_set_y(from._internal_y());
  }
  _this->_internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
}

void CBS_State::CopyFrom(const CBS_State& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:CBSProto.CBS.State)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool CBS_State::IsInitialized() const {
  return true;
}

void CBS_State::InternalSwap(CBS_State* other) {
  using std::swap;
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::internal::memswap<
      PROTOBUF_FIELD_OFFSET(CBS_State, _impl_.y_)
      + sizeof(CBS_State::_impl_.y_)
      - PROTOBUF_FIELD_OFFSET(CBS_State, _impl_.time_)>(
          reinterpret_cast<char*>(&_impl_.time_),
          reinterpret_cast<char*>(&other->_impl_.time_));
}

::PROTOBUF_NAMESPACE_ID::Metadata CBS_State::GetMetadata() const {
  return ::_pbi::AssignDescriptors(
      &descriptor_table_cbs_2eproto_getter, &descriptor_table_cbs_2eproto_once,
      file_level_metadata_cbs_2eproto[1]);
}

// ===================================================================

class CBS::_Internal {
 public:
};

CBS::CBS(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                         bool is_message_owned)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena, is_message_owned) {
  SharedCtor(arena, is_message_owned);
  // @@protoc_insertion_point(arena_constructor:CBSProto.CBS)
}
CBS::CBS(const CBS& from)
  : ::PROTOBUF_NAMESPACE_ID::Message() {
  CBS* const _this = this; (void)_this;
  new (&_impl_) Impl_{
      decltype(_impl_.obstacles_){from._impl_.obstacles_}
    , decltype(_impl_.goals_){from._impl_.goals_}
    , decltype(_impl_.start_states_){from._impl_.start_states_}
    , decltype(_impl_.num_of_rows_){}
    , decltype(_impl_.num_of_cols_){}
    , decltype(_impl_.world_size_){}
    , decltype(_impl_.world_rank_){}
    , /*decltype(_impl_._cached_size_)*/{}};

  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  ::memcpy(&_impl_.num_of_rows_, &from._impl_.num_of_rows_,
    static_cast<size_t>(reinterpret_cast<char*>(&_impl_.world_rank_) -
    reinterpret_cast<char*>(&_impl_.num_of_rows_)) + sizeof(_impl_.world_rank_));
  // @@protoc_insertion_point(copy_constructor:CBSProto.CBS)
}

inline void CBS::SharedCtor(
    ::_pb::Arena* arena, bool is_message_owned) {
  (void)arena;
  (void)is_message_owned;
  new (&_impl_) Impl_{
      decltype(_impl_.obstacles_){arena}
    , decltype(_impl_.goals_){arena}
    , decltype(_impl_.start_states_){arena}
    , decltype(_impl_.num_of_rows_){0}
    , decltype(_impl_.num_of_cols_){0}
    , decltype(_impl_.world_size_){0}
    , decltype(_impl_.world_rank_){0}
    , /*decltype(_impl_._cached_size_)*/{}
  };
}

CBS::~CBS() {
  // @@protoc_insertion_point(destructor:CBSProto.CBS)
  if (auto *arena = _internal_metadata_.DeleteReturnArena<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>()) {
  (void)arena;
    return;
  }
  SharedDtor();
}

inline void CBS::SharedDtor() {
  GOOGLE_DCHECK(GetArenaForAllocation() == nullptr);
  _impl_.obstacles_.~RepeatedPtrField();
  _impl_.goals_.~RepeatedPtrField();
  _impl_.start_states_.~RepeatedPtrField();
}

void CBS::SetCachedSize(int size) const {
  _impl_._cached_size_.Set(size);
}

void CBS::Clear() {
// @@protoc_insertion_point(message_clear_start:CBSProto.CBS)
  uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  _impl_.obstacles_.Clear();
  _impl_.goals_.Clear();
  _impl_.start_states_.Clear();
  ::memset(&_impl_.num_of_rows_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&_impl_.world_rank_) -
      reinterpret_cast<char*>(&_impl_.num_of_rows_)) + sizeof(_impl_.world_rank_));
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* CBS::_InternalParse(const char* ptr, ::_pbi::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  while (!ctx->Done(&ptr)) {
    uint32_t tag;
    ptr = ::_pbi::ReadTag(ptr, &tag);
    switch (tag >> 3) {
      // int32 num_of_rows = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 8)) {
          _impl_.num_of_rows_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint32(&ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // int32 num_of_cols = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 16)) {
          _impl_.num_of_cols_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint32(&ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // int32 world_size = 3;
      case 3:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 24)) {
          _impl_.world_size_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint32(&ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // int32 world_rank = 4;
      case 4:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 32)) {
          _impl_.world_rank_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint32(&ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // repeated .CBSProto.CBS.Location obstacles = 5;
      case 5:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 42)) {
          ptr -= 1;
          do {
            ptr += 1;
            ptr = ctx->ParseMessage(_internal_add_obstacles(), ptr);
            CHK_(ptr);
            if (!ctx->DataAvailable(ptr)) break;
          } while (::PROTOBUF_NAMESPACE_ID::internal::ExpectTag<42>(ptr));
        } else
          goto handle_unusual;
        continue;
      // repeated .CBSProto.CBS.Location goals = 6;
      case 6:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 50)) {
          ptr -= 1;
          do {
            ptr += 1;
            ptr = ctx->ParseMessage(_internal_add_goals(), ptr);
            CHK_(ptr);
            if (!ctx->DataAvailable(ptr)) break;
          } while (::PROTOBUF_NAMESPACE_ID::internal::ExpectTag<50>(ptr));
        } else
          goto handle_unusual;
        continue;
      // repeated .CBSProto.CBS.State start_states = 7;
      case 7:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 58)) {
          ptr -= 1;
          do {
            ptr += 1;
            ptr = ctx->ParseMessage(_internal_add_start_states(), ptr);
            CHK_(ptr);
            if (!ctx->DataAvailable(ptr)) break;
          } while (::PROTOBUF_NAMESPACE_ID::internal::ExpectTag<58>(ptr));
        } else
          goto handle_unusual;
        continue;
      default:
        goto handle_unusual;
    }  // switch
  handle_unusual:
    if ((tag == 0) || ((tag & 7) == 4)) {
      CHK_(ptr);
      ctx->SetLastTag(tag);
      goto message_done;
    }
    ptr = UnknownFieldParse(
        tag,
        _internal_metadata_.mutable_unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(),
        ptr, ctx);
    CHK_(ptr != nullptr);
  }  // while
message_done:
  return ptr;
failure:
  ptr = nullptr;
  goto message_done;
#undef CHK_
}

uint8_t* CBS::_InternalSerialize(
    uint8_t* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:CBSProto.CBS)
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  // int32 num_of_rows = 1;
  if (this->_internal_num_of_rows() != 0) {
    target = stream->EnsureSpace(target);
    target = ::_pbi::WireFormatLite::WriteInt32ToArray(1, this->_internal_num_of_rows(), target);
  }

  // int32 num_of_cols = 2;
  if (this->_internal_num_of_cols() != 0) {
    target = stream->EnsureSpace(target);
    target = ::_pbi::WireFormatLite::WriteInt32ToArray(2, this->_internal_num_of_cols(), target);
  }

  // int32 world_size = 3;
  if (this->_internal_world_size() != 0) {
    target = stream->EnsureSpace(target);
    target = ::_pbi::WireFormatLite::WriteInt32ToArray(3, this->_internal_world_size(), target);
  }

  // int32 world_rank = 4;
  if (this->_internal_world_rank() != 0) {
    target = stream->EnsureSpace(target);
    target = ::_pbi::WireFormatLite::WriteInt32ToArray(4, this->_internal_world_rank(), target);
  }

  // repeated .CBSProto.CBS.Location obstacles = 5;
  for (unsigned i = 0,
      n = static_cast<unsigned>(this->_internal_obstacles_size()); i < n; i++) {
    const auto& repfield = this->_internal_obstacles(i);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
        InternalWriteMessage(5, repfield, repfield.GetCachedSize(), target, stream);
  }

  // repeated .CBSProto.CBS.Location goals = 6;
  for (unsigned i = 0,
      n = static_cast<unsigned>(this->_internal_goals_size()); i < n; i++) {
    const auto& repfield = this->_internal_goals(i);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
        InternalWriteMessage(6, repfield, repfield.GetCachedSize(), target, stream);
  }

  // repeated .CBSProto.CBS.State start_states = 7;
  for (unsigned i = 0,
      n = static_cast<unsigned>(this->_internal_start_states_size()); i < n; i++) {
    const auto& repfield = this->_internal_start_states(i);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
        InternalWriteMessage(7, repfield, repfield.GetCachedSize(), target, stream);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::_pbi::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:CBSProto.CBS)
  return target;
}

size_t CBS::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:CBSProto.CBS)
  size_t total_size = 0;

  uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated .CBSProto.CBS.Location obstacles = 5;
  total_size += 1UL * this->_internal_obstacles_size();
  for (const auto& msg : this->_impl_.obstacles_) {
    total_size +=
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(msg);
  }

  // repeated .CBSProto.CBS.Location goals = 6;
  total_size += 1UL * this->_internal_goals_size();
  for (const auto& msg : this->_impl_.goals_) {
    total_size +=
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(msg);
  }

  // repeated .CBSProto.CBS.State start_states = 7;
  total_size += 1UL * this->_internal_start_states_size();
  for (const auto& msg : this->_impl_.start_states_) {
    total_size +=
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(msg);
  }

  // int32 num_of_rows = 1;
  if (this->_internal_num_of_rows() != 0) {
    total_size += ::_pbi::WireFormatLite::Int32SizePlusOne(this->_internal_num_of_rows());
  }

  // int32 num_of_cols = 2;
  if (this->_internal_num_of_cols() != 0) {
    total_size += ::_pbi::WireFormatLite::Int32SizePlusOne(this->_internal_num_of_cols());
  }

  // int32 world_size = 3;
  if (this->_internal_world_size() != 0) {
    total_size += ::_pbi::WireFormatLite::Int32SizePlusOne(this->_internal_world_size());
  }

  // int32 world_rank = 4;
  if (this->_internal_world_rank() != 0) {
    total_size += ::_pbi::WireFormatLite::Int32SizePlusOne(this->_internal_world_rank());
  }

  return MaybeComputeUnknownFieldsSize(total_size, &_impl_._cached_size_);
}

const ::PROTOBUF_NAMESPACE_ID::Message::ClassData CBS::_class_data_ = {
    ::PROTOBUF_NAMESPACE_ID::Message::CopyWithSourceCheck,
    CBS::MergeImpl
};
const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*CBS::GetClassData() const { return &_class_data_; }


void CBS::MergeImpl(::PROTOBUF_NAMESPACE_ID::Message& to_msg, const ::PROTOBUF_NAMESPACE_ID::Message& from_msg) {
  auto* const _this = static_cast<CBS*>(&to_msg);
  auto& from = static_cast<const CBS&>(from_msg);
  // @@protoc_insertion_point(class_specific_merge_from_start:CBSProto.CBS)
  GOOGLE_DCHECK_NE(&from, _this);
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  _this->_impl_.obstacles_.MergeFrom(from._impl_.obstacles_);
  _this->_impl_.goals_.MergeFrom(from._impl_.goals_);
  _this->_impl_.start_states_.MergeFrom(from._impl_.start_states_);
  if (from._internal_num_of_rows() != 0) {
    _this->_internal_set_num_of_rows(from._internal_num_of_rows());
  }
  if (from._internal_num_of_cols() != 0) {
    _this->_internal_set_num_of_cols(from._internal_num_of_cols());
  }
  if (from._internal_world_size() != 0) {
    _this->_internal_set_world_size(from._internal_world_size());
  }
  if (from._internal_world_rank() != 0) {
    _this->_internal_set_world_rank(from._internal_world_rank());
  }
  _this->_internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
}

void CBS::CopyFrom(const CBS& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:CBSProto.CBS)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool CBS::IsInitialized() const {
  return true;
}

void CBS::InternalSwap(CBS* other) {
  using std::swap;
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  _impl_.obstacles_.InternalSwap(&other->_impl_.obstacles_);
  _impl_.goals_.InternalSwap(&other->_impl_.goals_);
  _impl_.start_states_.InternalSwap(&other->_impl_.start_states_);
  ::PROTOBUF_NAMESPACE_ID::internal::memswap<
      PROTOBUF_FIELD_OFFSET(CBS, _impl_.world_rank_)
      + sizeof(CBS::_impl_.world_rank_)
      - PROTOBUF_FIELD_OFFSET(CBS, _impl_.num_of_rows_)>(
          reinterpret_cast<char*>(&_impl_.num_of_rows_),
          reinterpret_cast<char*>(&other->_impl_.num_of_rows_));
}

::PROTOBUF_NAMESPACE_ID::Metadata CBS::GetMetadata() const {
  return ::_pbi::AssignDescriptors(
      &descriptor_table_cbs_2eproto_getter, &descriptor_table_cbs_2eproto_once,
      file_level_metadata_cbs_2eproto[2]);
}

// @@protoc_insertion_point(namespace_scope)
}  // namespace CBSProto
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::CBSProto::CBS_Location*
Arena::CreateMaybeMessage< ::CBSProto::CBS_Location >(Arena* arena) {
  return Arena::CreateMessageInternal< ::CBSProto::CBS_Location >(arena);
}
template<> PROTOBUF_NOINLINE ::CBSProto::CBS_State*
Arena::CreateMaybeMessage< ::CBSProto::CBS_State >(Arena* arena) {
  return Arena::CreateMessageInternal< ::CBSProto::CBS_State >(arena);
}
template<> PROTOBUF_NOINLINE ::CBSProto::CBS*
Arena::CreateMaybeMessage< ::CBSProto::CBS >(Arena* arena) {
  return Arena::CreateMessageInternal< ::CBSProto::CBS >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
