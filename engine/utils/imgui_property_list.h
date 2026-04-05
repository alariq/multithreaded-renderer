#pragma once

#include "engine/utils/vec.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cassert>
#include <iterator>
#include <sstream>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>
#include <cfloat> // FLT_MIN/MAX

#if defined(USE_IMGUI)
#include "imgui.h"
#endif

namespace imgui_props {

enum class PropertyKind {
    Section,
    ReadOnlyText,
    Bool,
    Int,
    UInt,
    Float,
    Vec2,
    Vec3,
    Vec4,
    ChildObject,
    ChildArray
};

template <typename TObject>
bool DrawPropertySheetTable(const char* id, TObject& object);
template <typename TObject>
bool DrawPropertyRowsWithBases(TObject& object, int tree_depth);
template <typename TObject>
struct PropertyList;
template <typename TObject>
const PropertyList<TObject>& GetPropertyList();

template <typename TContainer, typename TLabelFn>
bool DrawPropertySheetFlatList(const char* id, TContainer& items, TLabelFn&& label_fn);
template <typename TContainer>
bool DrawPropertySheetFlatList(const char* id, TContainer& items);
template <typename TContainer, typename TLabelFn>
bool DrawPropertySheetCollapsibleList(const char* id, TContainer& items, TLabelFn&& label_fn);
template <typename TContainer>
bool DrawPropertySheetCollapsibleList(const char* id, TContainer& items);

struct IPolymorphicPropertyObject {
    virtual ~IPolymorphicPropertyObject() = default;
    virtual bool DrawPolymorphicPropertySheet(const char* id) { return false; }
    virtual bool DrawPolymorphicPropertyRows(int tree_depth)
    {
        (void)tree_depth;
        return false;
    }
    virtual const char* GetPolymorphicPropertyTypeName() const { return nullptr; }
};

template <typename TObject>
struct HasPropertyList : std::false_type {};

template <typename TObject>
struct PropertyBaseType {
    using type = void;
};

enum PropertyFlags : uint32_t {
    kPropertyFlagNone = 0u,
    kPropertyFlagReadOnly = 1u << 0
};

namespace detail {

template <typename TObject>
inline bool draw_registered_property_sheet(const char* id, TObject& object)
{
    if constexpr (HasPropertyList<TObject>::value) {
        return DrawPropertySheetTable(id, object);
    } else {
        (void)id;
        (void)object;
        return false;
    }
}

template <typename TObject>
inline bool draw_registered_property_rows(TObject& object, int tree_depth)
{
    if constexpr (HasPropertyList<TObject>::value) {
        return DrawPropertyRowsWithBases(object, tree_depth);
    } else {
        (void)object;
        (void)tree_depth;
        return false;
    }
}

template <typename TObject>
inline const char* get_registered_type_name()
{
    if constexpr (HasPropertyList<TObject>::value) {
        const char* type_name = GetPropertyList<TObject>().type_name;
        return (type_name && type_name[0] != '\0') ? type_name : nullptr;
    } else {
        return nullptr;
    }
}

template <typename TElement>
inline const char* resolve_element_type_name(TElement& element)
{
    using ElementType = std::remove_cv_t<std::remove_reference_t<TElement>>;
    if constexpr (std::is_pointer<ElementType>::value) {
        if (!element) {
            return nullptr;
        }
        using PointeeType = std::remove_pointer_t<ElementType>;
        if constexpr (std::is_polymorphic<PointeeType>::value) {
            if (auto* polymorphic = dynamic_cast<IPolymorphicPropertyObject*>(element)) {
                const char* dynamic_name = polymorphic->GetPolymorphicPropertyTypeName();
                if (dynamic_name && dynamic_name[0] != '\0') {
                    return dynamic_name;
                }
            }
        }
        return get_registered_type_name<PointeeType>();
    } else {
        if constexpr (std::is_polymorphic<ElementType>::value) {
            if (auto* polymorphic = dynamic_cast<IPolymorphicPropertyObject*>(&element)) {
                const char* dynamic_name = polymorphic->GetPolymorphicPropertyTypeName();
                if (dynamic_name && dynamic_name[0] != '\0') {
                    return dynamic_name;
                }
            }
        }
        return get_registered_type_name<ElementType>();
    }
}

template <typename TValue>
inline TValue clamp_value(TValue value, TValue min_value, TValue max_value)
{
    return std::max(min_value, std::min(value, max_value));
}

inline std::string to_string_value(const std::string& value)
{
    return value;
}

inline std::string to_string_value(const char* value)
{
    return value ? std::string(value) : std::string();
}

inline std::string to_string_value(char* value)
{
    return value ? std::string(value) : std::string();
}

inline std::string to_string_value(bool value)
{
    return value ? "true" : "false";
}

template <typename TValue>
inline std::string to_string_value(const TValue& value)
{
    std::ostringstream out;
    out << value;
    return out.str();
}

template <typename TLabel>
inline std::string normalize_label(TLabel&& label)
{
    using LabelType = std::decay_t<TLabel>;
    if constexpr (std::is_same<LabelType, std::string>::value) {
        return std::forward<TLabel>(label);
    } else if constexpr (std::is_same<LabelType, const char*>::value || std::is_same<LabelType, char*>::value) {
        const char* text = label;
        return text ? std::string(text) : std::string();
    } else {
        return to_string_value(label);
    }
}

template <typename TObject, typename TChild, TChild TObject::* Member>
inline void* child_object_get(TObject& object)
{
    return &(object.*Member);
}

template <typename TObject, typename TChild, TChild* TObject::* Member>
inline void* child_pointer_get(TObject& object)
{
    return object.*Member;
}

template <typename TChild>
inline bool child_draw_sheet(const char* id, void* child_ptr)
{
    if (!child_ptr) {
        return false;
    }
    TChild& child = *static_cast<TChild*>(child_ptr);
    if constexpr (std::is_polymorphic<TChild>::value) {
        if (auto* polymorphic = dynamic_cast<IPolymorphicPropertyObject*>(&child)) {
            return polymorphic->DrawPolymorphicPropertySheet(id);
        }
    }
    return draw_registered_property_sheet(id, child);
}

template <typename TChild>
inline bool child_draw_rows(void* child_ptr, int tree_depth)
{
    if (!child_ptr) {
        return false;
    }
    TChild& child = *static_cast<TChild*>(child_ptr);
    if constexpr (std::is_polymorphic<TChild>::value) {
        if (auto* polymorphic = dynamic_cast<IPolymorphicPropertyObject*>(&child)) {
            return polymorphic->DrawPolymorphicPropertyRows(tree_depth);
        }
    }
    return draw_registered_property_rows(child, tree_depth);
}

template <typename TElement>
inline bool draw_element_sheet(const char* id, TElement& element)
{
    using ElementType = std::remove_cv_t<std::remove_reference_t<TElement>>;
    if constexpr (std::is_pointer<ElementType>::value) {
        if (!element) {
            return false;
        }
        using PointeeType = std::remove_pointer_t<ElementType>;
        if constexpr (std::is_polymorphic<PointeeType>::value) {
            if (auto* polymorphic = dynamic_cast<IPolymorphicPropertyObject*>(element)) {
                return polymorphic->DrawPolymorphicPropertySheet(id);
            }
        }
        return draw_registered_property_sheet(id, *element);
    } else {
        if constexpr (std::is_polymorphic<ElementType>::value) {
            if (auto* polymorphic = dynamic_cast<IPolymorphicPropertyObject*>(&element)) {
                return polymorphic->DrawPolymorphicPropertySheet(id);
            }
        }
        return draw_registered_property_sheet(id, element);
    }
}

template <typename TContainer>
inline bool iterable_is_empty(TContainer& items)
{
    return std::begin(items) == std::end(items);
}

template <typename TContainer>
inline bool draw_container_sheet(const char* id, TContainer& items, bool collapsible)
{
    if (collapsible) {
        return DrawPropertySheetCollapsibleList(id, items);
    }
    return DrawPropertySheetFlatList(id, items);
}

template <typename TElement>
inline bool draw_element_rows(const char* label, TElement& element, int tree_depth)
{
#if defined(USE_IMGUI)
    using ElementType = std::remove_cv_t<std::remove_reference_t<TElement>>;

    ImGui::TableNextRow();
    ImGui::TableSetColumnIndex(0);

    bool changed = false;
    bool has_child = true;

    if constexpr (std::is_pointer<ElementType>::value) {
        if (!element) {
            has_child = false;
        }
    }

    if (!has_child) {
        ImGui::TreeNodeEx(label, ImGuiTreeNodeFlags_Leaf | ImGuiTreeNodeFlags_NoTreePushOnOpen);
        ImGui::TableSetColumnIndex(1);
        ImGui::TextUnformatted("<null>");
        return false;
    }

    const bool open = ImGui::TreeNodeEx(label, ImGuiTreeNodeFlags_DefaultOpen);
    ImGui::TableSetColumnIndex(1);
    const char* type_name = resolve_element_type_name(element);
    if (type_name && type_name[0] != '\0') {
        ImGui::TextUnformatted(type_name);
    }

    if (open) {
        if constexpr (std::is_pointer<ElementType>::value) {
            using PointeeType = std::remove_pointer_t<ElementType>;
            if constexpr (std::is_polymorphic<PointeeType>::value) {
                if (auto* polymorphic = dynamic_cast<IPolymorphicPropertyObject*>(element)) {
                    changed = polymorphic->DrawPolymorphicPropertyRows(tree_depth + 1) || changed;
                } else {
                    changed = draw_registered_property_rows(*element, tree_depth + 1) || changed;
                }
            } else {
                changed = draw_registered_property_rows(*element, tree_depth + 1) || changed;
            }
        } else {
            if constexpr (std::is_polymorphic<ElementType>::value) {
                if (auto* polymorphic = dynamic_cast<IPolymorphicPropertyObject*>(&element)) {
                    changed = polymorphic->DrawPolymorphicPropertyRows(tree_depth + 1) || changed;
                } else {
                    changed = draw_registered_property_rows(element, tree_depth + 1) || changed;
                }
            } else {
                changed = draw_registered_property_rows(element, tree_depth + 1) || changed;
            }
        }
        ImGui::TreePop();
    }

    return changed;
#else
    (void)label;
    (void)element;
    (void)tree_depth;
    return false;
#endif
}

template <typename TContainer>
inline bool draw_container_rows(TContainer& items, int tree_depth, const char* empty_text)
{
#if defined(USE_IMGUI)
    if (iterable_is_empty(items)) {
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::TextUnformatted(empty_text ? empty_text : "<empty>");
        ImGui::TableSetColumnIndex(1);
        return false;
    }

    bool changed = false;
    size_t index = 0;
    for (auto& item : items) {
        ImGui::PushID(static_cast<int>(index));
        const std::string label = std::string("Item ") + std::to_string(index);
        changed = draw_element_rows(label.c_str(), item, tree_depth) || changed;
        ImGui::PopID();
        ++index;
    }

    return changed;
#else
    (void)items;
    (void)tree_depth;
    (void)empty_text;
    return false;
#endif
}

template <typename TObject, typename TContainer, TContainer TObject::* Member>
inline bool child_array_draw(const char* id,
                             TObject& object,
                             bool collapsible,
                             const char* empty_text,
                             const char*)
{
    TContainer& items = object.*Member;
    if (iterable_is_empty(items)) {
#if defined(USE_IMGUI)
        ImGui::TextUnformatted(empty_text ? empty_text : "<empty>");
#endif
        return false;
    }
    return draw_container_sheet(id, items, collapsible);
}

template <typename TObject, typename TContainer, TContainer TObject::* Member>
inline bool child_array_draw_rows(TObject& object,
                                  int tree_depth,
                                  const char* empty_text,
                                  const char*)
{
    TContainer& items = object.*Member;
    return draw_container_rows(items, tree_depth, empty_text);
}

template <typename TObject, typename TContainer, TContainer* TObject::* Member>
inline bool child_array_pointer_draw(const char* id,
                                     TObject& object,
                                     bool collapsible,
                                     const char* empty_text,
                                     const char* null_text)
{
    TContainer* items = object.*Member;
    if (!items) {
#if defined(USE_IMGUI)
        ImGui::TextUnformatted(null_text ? null_text : "<null>");
#endif
        return false;
    }
    if (iterable_is_empty(*items)) {
#if defined(USE_IMGUI)
        ImGui::TextUnformatted(empty_text ? empty_text : "<empty>");
#endif
        return false;
    }
    return draw_container_sheet(id, *items, collapsible);
}

template <typename TObject, typename TContainer, TContainer* TObject::* Member>
inline bool child_array_pointer_draw_rows(TObject& object,
                                          int tree_depth,
                                          const char* empty_text,
                                          const char* null_text)
{
    TContainer* items = object.*Member;
#if defined(USE_IMGUI)
    if (!items) {
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::TextUnformatted(null_text ? null_text : "<null>");
        ImGui::TableSetColumnIndex(1);
        return false;
    }
#else
    (void)tree_depth;
    (void)null_text;
#endif
    if (!items) {
        return false;
    }
    return draw_container_rows(*items, tree_depth, empty_text);
}

} // namespace detail

template <typename TObject>
struct PropertyDesc {
    using TextGetter = std::string (*)(const TObject&);
    using TextGetterCStr = const char* (*)(const TObject&);

    using BoolGetter = bool (*)(const TObject&);
    using BoolSetter = void (*)(TObject&, bool);

    using IntGetter = int (*)(const TObject&);
    using IntSetter = void (*)(TObject&, int);

    using UIntGetter = unsigned int (*)(const TObject&);
    using UIntSetter = void (*)(TObject&, unsigned int);

    using FloatGetter = float (*)(const TObject&);
    using FloatSetter = void (*)(TObject&, float);

    using Vec2Getter = vec2 (*)(const TObject&);
    using Vec2Setter = void (*)(TObject&, const vec2&);

    using Vec3Getter = vec3 (*)(const TObject&);
    using Vec3Setter = void (*)(TObject&, const vec3&);

    using Vec4Getter = vec4 (*)(const TObject&);
    using Vec4Setter = void (*)(TObject&, const vec4&);

    using ChildGetter = void* (*)(TObject&);
    using ChildDrawer = bool (*)(const char* id, void* child_ptr);
    using ChildRowsDrawer = bool (*)(void* child_ptr, int tree_depth);
    using ChildArrayDrawer = bool (*)(const char* id,
                                      TObject& object,
                                      bool collapsible,
                                      const char* empty_text,
                                      const char* null_text);
    using ChildArrayRowsDrawer = bool (*)(TObject& object,
                                          int tree_depth,
                                          const char* empty_text,
                                          const char* null_text);

    const char* label = "";
    PropertyKind kind = PropertyKind::Section;
    uint32_t flags = kPropertyFlagNone;

    union {
        struct {
            int iminv, imaxv, istep;
        };
        struct {
            float fminv, fmaxv, fstep;
        };
    } meta;

    // cannot add both get text types to a union because we may call wrong ones as they are not mutually exclusive
    // better way: have 2 distinct types, or one getter and save type info anyway to do a proper cast when calling
    // during the draw stage
    TextGetter get_text = nullptr;
    union {
        TextGetterCStr get_text_cstr;
        BoolGetter get_bool;
        IntGetter get_int;
        UIntGetter get_uint;
        FloatGetter get_float;
        Vec2Getter get_vec2;
        Vec3Getter get_vec3;
        Vec4Getter get_vec4 = nullptr;
    };
    
    union {
        BoolSetter set_bool;
        IntSetter set_int;
        UIntSetter set_uint;
        FloatSetter set_float;
        Vec2Setter set_vec2;
        Vec3Setter set_vec3;
        Vec4Setter set_vec4 = nullptr;
    };
    
    union {
        bool TObject::* bool_member;
        int TObject::* int_member;
        uint TObject::* uint_member;
        float TObject::* float_member;
        vec2 TObject::* vec2_member;
        vec3 TObject::* vec3_member;
        vec4 TObject::* vec4_member = nullptr;
    };

    ChildGetter get_child = nullptr;
    ChildDrawer draw_child = nullptr;
    ChildRowsDrawer draw_child_rows = nullptr;
    bool child_collapsible = true;
    const char* child_null_text = "<null>";

    ChildArrayDrawer draw_child_array = nullptr;
    ChildArrayRowsDrawer draw_child_array_rows = nullptr;
    bool array_collapsible = true;

    const char* array_empty_text = "<empty>";
    const char* array_null_text = "<null>";
};

template <typename TObject>
struct PropertyList {
    const char* type_name = "";
    std::vector<PropertyDesc<TObject>> items;
};

template <typename TObject>
class PropertyListBuilder {
public:
    PropertyListBuilder& TypeName(const char* type_name)
    {
        list_.type_name = type_name ? type_name : "";
        return *this;
    }

    PropertyListBuilder& Section(const char* label)
    {
        PropertyDesc<TObject> desc;
        desc.label = label;
        desc.kind = PropertyKind::Section;
        list_.items.push_back(desc);
        return *this;
    }

    PropertyListBuilder& ReadOnlyText(const char* label, typename PropertyDesc<TObject>::TextGetter getter)
    {
        PropertyDesc<TObject> desc;
        desc.label = label;
        desc.kind = PropertyKind::ReadOnlyText;
        desc.flags = kPropertyFlagReadOnly;
        desc.get_text = getter;
        list_.items.push_back(desc);
        return *this;
    }

    PropertyListBuilder& ReadOnlyText(const char* label, typename PropertyDesc<TObject>::TextGetterCStr getter)
    {
        PropertyDesc<TObject> desc;
        desc.label = label;
        desc.kind = PropertyKind::ReadOnlyText;
        desc.flags = kPropertyFlagReadOnly;
        desc.get_text_cstr = getter;
        list_.items.push_back(desc);
        return *this;
    }

    PropertyListBuilder& Bool(const char* label,
                              bool TObject::* member,
                              uint32_t flags = 0,
                              typename PropertyDesc<TObject>::BoolGetter getter = nullptr,
                              typename PropertyDesc<TObject>::BoolSetter setter = nullptr)
    {
        assert(member || (getter && setter));
        PropertyDesc<TObject> desc;
        desc.label = label;
        desc.kind = PropertyKind::Bool;
        desc.flags = flags;
        desc.bool_member = member;
        desc.get_bool = getter;
        desc.set_bool = setter;
        list_.items.push_back(desc);
        return *this;
    }

    PropertyListBuilder& Int(const char* label,
                             int TObject::* member,
                             uint32_t flags = 0,
                             int min_value = INT32_MIN,
                             int max_value = INT32_MAX,
                             int step = 1,
                             typename PropertyDesc<TObject>::IntGetter getter = nullptr,
                             typename PropertyDesc<TObject>::IntSetter setter = nullptr)
    {
        PropertyDesc<TObject> desc;
        desc.label = label;
        desc.kind = PropertyKind::Int;
        desc.flags = flags;
        desc.meta.iminv = min_value;
        desc.meta.imaxv = max_value;
        desc.meta.istep = step;
        desc.int_member = member;
        desc.get_int = getter;
        desc.set_int = setter;
        list_.items.push_back(desc);
        return *this;
    }

    PropertyListBuilder& Int(const char* label,
                             int const TObject::* member,
                             uint32_t flags = 0,
                             int min_value = INT32_MIN,
                             int max_value = INT32_MAX,
                             int step = 1,
                             typename PropertyDesc<TObject>::IntGetter getter = nullptr,
                             typename PropertyDesc<TObject>::IntSetter setter = nullptr)
    {
        return Int(label, const_cast<int TObject::*>(member), 
                PropertyFlags::kPropertyFlagReadOnly, min_value, max_value, 1, getter, setter);
    }


    PropertyListBuilder& UInt(const char* label,
                             unsigned int TObject::* member,
                             int min_value = 0,
                             int max_value = UINT32_MAX,
                             int step = 1,
                             uint32_t flags = 0,
                             typename PropertyDesc<TObject>::UIntGetter getter = nullptr,
                             typename PropertyDesc<TObject>::UIntSetter setter = nullptr)
    {
        PropertyDesc<TObject> desc;
        desc.label = label;
        desc.kind = PropertyKind::UInt;
        desc.flags = flags;
        desc.meta.iminv = min_value;
        desc.meta.imaxv = max_value;
        desc.meta.istep = step;
        desc.uint_member = member;
        desc.get_uint = getter;
        desc.set_uint = setter;
        list_.items.push_back(desc);
        return *this;
    }

    PropertyListBuilder& UInt(const char* label,
                             unsigned int const TObject::* member,
                             int min_value = 0,
                             int max_value = UINT32_MAX,
                             int step = 1,
                             uint32_t flags = 0,
                             typename PropertyDesc<TObject>::UIntGetter getter = nullptr,
                             typename PropertyDesc<TObject>::UIntSetter setter = nullptr)
    {
        return UInt(label, const_cast<unsigned int TObject::*>(member),
                PropertyFlags::kPropertyFlagReadOnly, min_value, max_value, 1, getter, setter);
    }


    PropertyListBuilder& Float(const char* label,
                               float TObject::* member,
                               uint32_t flags = 0,
                               float min_value = -FLT_MAX,
                               float max_value = FLT_MAX,
                               float step = 1,
                               typename PropertyDesc<TObject>::FloatGetter getter = nullptr,
                               typename PropertyDesc<TObject>::FloatSetter setter = nullptr)
    {
        assert(member || (getter && setter));
        PropertyDesc<TObject> desc;
        desc.label = label;
        desc.kind = PropertyKind::Float;
        desc.flags = flags;
        desc.meta.fminv = min_value;
        desc.meta.fmaxv = max_value;
        desc.meta.fstep = step;
        desc.float_member = member;
        desc.get_float = getter;
        desc.set_float = setter;
        list_.items.push_back(desc);
        return *this;
    }

    PropertyListBuilder& Float(const char* label,
                               const float TObject::* member,
                               float min_value = -FLT_MAX,
                               float max_value = FLT_MAX,
                               float step = 1,
                               typename PropertyDesc<TObject>::FloatGetter getter = nullptr,
                               typename PropertyDesc<TObject>::FloatSetter setter = nullptr)
    {
        return Float(label, const_cast<float TObject::*>(member), 
                PropertyFlags::kPropertyFlagReadOnly, min_value, max_value, 1, getter, setter);
    }

    PropertyListBuilder& Vec2(const char* label,
                              vec2 TObject::* member,
                              uint32_t flags = 0,
                              float step = 1.0f,
                              typename PropertyDesc<TObject>::Vec2Getter getter = nullptr,
                              typename PropertyDesc<TObject>::Vec2Setter setter = nullptr)
    {
        PropertyDesc<TObject> desc;
        desc.label = label;
        desc.kind = PropertyKind::Vec2;
        desc.flags = flags;
        desc.meta.fstep = step;
        desc.vec2_member = member;
        desc.get_vec2 = getter;
        desc.set_vec2 = setter;
        list_.items.push_back(desc);
        return *this;
    }

    PropertyListBuilder& Vec2Accessor(const char* label,
                                      typename PropertyDesc<TObject>::Vec2Getter getter,
                                      typename PropertyDesc<TObject>::Vec2Setter setter,
                                      float step)
    {
        return Vec2(label, nullptr, 0, step, getter, setter);
    }

    PropertyListBuilder& Vec3(const char* label,
                              vec3 TObject::* member,
                              uint32_t flags = 0,
                              float step = 1.0f,
                              typename PropertyDesc<TObject>::Vec3Getter getter = nullptr,
                              typename PropertyDesc<TObject>::Vec3Setter setter = nullptr)
    {
        PropertyDesc<TObject> desc;
        desc.label = label;
        desc.kind = PropertyKind::Vec3;
        desc.flags = flags;
        desc.meta.fstep = step;
        desc.vec3_member = member;
        desc.get_vec3 = getter;
        desc.set_vec3 = setter;
        list_.items.push_back(desc);
        return *this;
    }

    PropertyListBuilder& Vec3Accessor(const char* label,
                                      typename PropertyDesc<TObject>::Vec3Getter getter,
                                      typename PropertyDesc<TObject>::Vec3Setter setter,
                                      float step)
    {
        return Vec3(label, nullptr, 0, step, getter, setter);
    }

    PropertyListBuilder& Vec4(const char* label,
                              vec4 TObject::* member,
                              uint32_t flags = 0,
                              float step = 1.0f,
                              typename PropertyDesc<TObject>::Vec4Getter getter = nullptr,
                              typename PropertyDesc<TObject>::Vec4Setter setter = nullptr)
    {
        PropertyDesc<TObject> desc;
        desc.label = label;
        desc.kind = PropertyKind::Vec4;
        desc.flags = flags;
        desc.meta.fstep = step;
        desc.vec4_member = member;
        desc.get_vec4 = getter;
        desc.set_vec4 = setter;
        list_.items.push_back(desc);
        return *this;
    }

    PropertyListBuilder& Vec4Accessor(const char* label,
                                      typename PropertyDesc<TObject>::Vec4Getter getter,
                                      typename PropertyDesc<TObject>::Vec4Setter setter,
                                      float step)
    {
        return Vec4(label, nullptr, 0, step, getter, setter);
    }

    template <typename TChild, TChild TObject::* Member>
    PropertyListBuilder& Child(const char* label, bool collapsible = true, const char* null_text = "<null>")
    {
        PropertyDesc<TObject> desc;
        desc.label = label;
        desc.kind = PropertyKind::ChildObject;
        desc.get_child = &detail::child_object_get<TObject, TChild, Member>;
        desc.draw_child = &detail::child_draw_sheet<TChild>;
        desc.draw_child_rows = &detail::child_draw_rows<TChild>;
        desc.child_collapsible = collapsible;
        desc.child_null_text = null_text;
        list_.items.push_back(desc);
        return *this;
    }

    template <typename TChild, TChild* TObject::* Member>
    PropertyListBuilder& ChildPtr(const char* label, bool collapsible = true, const char* null_text = "<null>")
    {
        PropertyDesc<TObject> desc;
        desc.label = label;
        desc.kind = PropertyKind::ChildObject;
        desc.get_child = &detail::child_pointer_get<TObject, TChild, Member>;
        desc.draw_child = &detail::child_draw_sheet<TChild>;
        desc.draw_child_rows = &detail::child_draw_rows<TChild>;
        desc.child_collapsible = collapsible;
        desc.child_null_text = null_text;
        list_.items.push_back(desc);
        return *this;
    }

    template <typename TContainer, TContainer TObject::* Member>
    PropertyListBuilder& Array(const char* label,
                               bool collapsible = true,
                               const char* empty_text = "<empty>")
    {
        PropertyDesc<TObject> desc;
        desc.label = label;
        desc.kind = PropertyKind::ChildArray;
        desc.draw_child_array = &detail::child_array_draw<TObject, TContainer, Member>;
        desc.draw_child_array_rows = &detail::child_array_draw_rows<TObject, TContainer, Member>;
        desc.array_collapsible = collapsible;
        desc.array_empty_text = empty_text;
        list_.items.push_back(desc);
        return *this;
    }

    template <typename TContainer, TContainer* TObject::* Member>
    PropertyListBuilder& ArrayPtr(const char* label,
                                  bool collapsible = true,
                                  const char* empty_text = "<empty>",
                                  const char* null_text = "<null>")
    {
        PropertyDesc<TObject> desc;
        desc.label = label;
        desc.kind = PropertyKind::ChildArray;
        desc.draw_child_array = &detail::child_array_pointer_draw<TObject, TContainer, Member>;
        desc.draw_child_array_rows = &detail::child_array_pointer_draw_rows<TObject, TContainer, Member>;
        desc.array_collapsible = collapsible;
        desc.array_empty_text = empty_text;
        desc.array_null_text = null_text;
        list_.items.push_back(desc);
        return *this;
    }

    PropertyList<TObject> Build() const
    {
        return list_;
    }

private:
    PropertyList<TObject> list_;
};

template <typename>
struct dependent_false : std::false_type {};

template <typename TObject>
const PropertyList<TObject>& GetPropertyList()
{
    static_assert(dependent_false<TObject>::value,
                  "GetPropertyList<T>() is not specialized. Use PROPERTY_LIST_BEGIN/END macros.");
    static PropertyList<TObject> empty;
    return empty;
}

#if defined(USE_IMGUI)
template <typename TObject>
bool DrawPropertyWidgetImpl(TObject& object, const PropertyDesc<TObject>& desc, bool hide_label)
{
    const bool is_read_only = (desc.flags & kPropertyFlagReadOnly) != 0u;
    const char* widget_label = hide_label ? "##value" : desc.label;

    ImGui::SetNextItemWidth(-FLT_MIN); // for widget to take whole cell of a table

    switch (desc.kind) {
    case PropertyKind::Section:
        if (!hide_label) {
            ImGui::TextUnformatted(desc.label ? desc.label : "");
            ImGui::Separator();
        }
        return false;

    case PropertyKind::ReadOnlyText: {
        if (!desc.get_text && !desc.get_text_cstr) {
            return false;
        }
        const std::string value = desc.get_text
            ? desc.get_text(object)
            : detail::to_string_value(desc.get_text_cstr(object));
        if (hide_label) {
            ImGui::TextUnformatted(value.c_str());
        } else {
            ImGui::Text("%s: %s", desc.label ? desc.label : "", value.c_str());
        }
        return false;
    }

    case PropertyKind::Bool: {
        bool value = false;
        if (desc.get_bool) {
            value = desc.get_bool(object);
        } else if (desc.bool_member) {
            value = object.*(desc.bool_member);
        } else {
            return false;
        }

        if (is_read_only) { ImGui::BeginDisabled(true); }
        const bool changed = ImGui::Checkbox(widget_label, &value);
        if (is_read_only) { ImGui::EndDisabled(); }
        if (changed && !is_read_only) {
            if (desc.set_bool) {
                desc.set_bool(object, value);
            } else if (desc.bool_member) {
                object.*(desc.bool_member) = value;
            } else {
                return false;
            }
            return true;
        }
        return false;
    }

    case PropertyKind::Int: {
        int value = 0;
        if (desc.get_int) {
            value = desc.get_int(object);
        } else if (desc.int_member) {
            value = object.*(desc.int_member);
        } else {
            return false;
        }

        const int step = desc.meta.istep;
        ImGuiInputTextFlags flags = is_read_only ? ImGuiInputTextFlags_ReadOnly : 0;
        const bool changed = ImGui::InputInt(widget_label, &value, step, 10*step, flags);

        if (changed && !is_read_only) {
            value = clamp(value, desc.meta.iminv, desc.meta.imaxv);
            if (desc.set_int) {
                desc.set_int(object, value);
            } else if (desc.int_member) {
                object.*(desc.int_member) = value;
            } else {
                return false;
            }
            return true;
        }
        return false;
    }

    case PropertyKind::UInt: {
        uint32_t value = 0;
        if (desc.get_uint) {
            value = desc.get_uint(object);
        } else if (desc.uint_member) {
            value = object.*(desc.uint_member);
        } else {
            return false;
        }

        const int step = desc.meta.istep;
        const int step_fast = 10*step;
        ImGuiInputTextFlags flags = is_read_only ? ImGuiInputTextFlags_ReadOnly : 0;
        //const bool changed = ImGui::InputInt(widget_label, &value, step, 10*step, flags);

        const char* format = (flags & ImGuiInputTextFlags_CharsHexadecimal) ? "%08X" : "%d";
        const bool changed = ImGui::InputScalar(widget_label, ImGuiDataType_U32, (void*)&value, &step, &step_fast, format, flags);

        if (changed && !is_read_only) {
            value = clamp(value, (uint32_t)desc.meta.iminv, (uint32_t)desc.meta.imaxv);
            if (desc.set_uint) {
                desc.set_uint(object, value);
            } else if (desc.uint_member) {
                object.*(desc.uint_member) = value;
            } else {
                return false;
            }
            return true;
        }
        return false;
    }

    case PropertyKind::Float: {
        float value = 0.0f;
        if (desc.get_float) {
            value = desc.get_float(object);
        } else if (desc.float_member) {
            value = object.*(desc.float_member);
        } else {
            return false;
        }

        const float step = desc.meta.fstep;
        ImGuiInputTextFlags flags = is_read_only ? ImGuiInputTextFlags_ReadOnly : 0;
        const bool changed = ImGui::InputFloat(widget_label, &value, step, step * 10.0f, "%.3f", flags);

        if (changed && !is_read_only) {
            value = clamp(value, desc.meta.fminv, desc.meta.fmaxv);
            if (desc.set_float) {
                desc.set_float(object, value);
            } else if (desc.float_member) {
                object.*(desc.float_member) = value;
            } else {
                return false;
            }
            return true;
        }
        return false;
    }

    case PropertyKind::Vec2: {
        vec2 value;
        if (desc.get_vec2) {
            value = desc.get_vec2(object);
        } else if (desc.vec2_member) {
            value = object.*(desc.vec2_member);
        } else {
            return false;
        }

        const float step = desc.meta.fstep;

        if (is_read_only) { ImGui::BeginDisabled(true); }
        const bool changed = ImGui::DragFloat2(widget_label, (float*)value, step);
        if (is_read_only) { ImGui::EndDisabled(); }

        if (changed && !is_read_only) {
            if (desc.set_vec2) {
                desc.set_vec2(object, value);
            } else if (desc.vec2_member) {
                object.*(desc.vec2_member) = value;
            } else {
                return false;
            }
            return true;
        }
        return false;
    }

    case PropertyKind::Vec3: {
        vec3 value;
        if (desc.get_vec3) {
            value = desc.get_vec3(object);
        } else if (desc.vec3_member) {
            value = object.*(desc.vec3_member);
        } else {
            return false;
        }

        const float step = desc.meta.fstep;

        if (is_read_only) { ImGui::BeginDisabled(true); }
        const bool changed = ImGui::DragFloat3(widget_label, (float*)value, step);
        if (is_read_only) { ImGui::EndDisabled(); }

        if (changed && !is_read_only) {
            if (desc.set_vec3) {
                desc.set_vec3(object, value);
            } else if (desc.vec3_member) {
                object.*(desc.vec3_member) = value;
            } else {
                return false;
            }
            return true;
        }
        return false;
    }
    case PropertyKind::Vec4: {
        vec4 value;
        if (desc.get_vec4) {
            value = desc.get_vec4(object);
        } else if (desc.vec4_member) {
            value = object.*(desc.vec4_member);
        } else {
            return false;
        }

        const float step = desc.meta.fstep;

        if (is_read_only) { ImGui::BeginDisabled(true); }
        const bool changed = ImGui::DragFloat4(widget_label, (float*)value, step);
        if (is_read_only) { ImGui::EndDisabled(); }

        if (changed && !is_read_only) {
            if (desc.set_vec4) {
                desc.set_vec4(object, value);
            } else if (desc.vec4_member) {
                object.*(desc.vec4_member) = value;
            } else {
                return false;
            }
            return true;
        }
        return false;
    }
    case PropertyKind::ChildObject: {
        if (!desc.get_child || !desc.draw_child) {
            return false;
        }

        void* child_ptr = desc.get_child(object);
        if (!child_ptr) {
            ImGui::TextUnformatted(desc.child_null_text ? desc.child_null_text : "<null>");
            return false;
        }

        if (desc.child_collapsible) {
            const bool is_open = ImGui::TreeNodeEx(widget_label);
            if (!is_open) {
                return false;
            }
            const bool changed = desc.draw_child("child_sheet", child_ptr);
            ImGui::TreePop();
            return changed;
        }

        return desc.draw_child("child_sheet", child_ptr);
    }
    case PropertyKind::ChildArray: {
        if (!desc.draw_child_array) {
            return false;
        }

        if (desc.array_collapsible) {
            const bool is_open = ImGui::TreeNodeEx(widget_label);
            if (!is_open) {
                return false;
            }
            const bool changed = desc.draw_child_array("child_array_sheet",
                                                       object,
                                                       true,
                                                       desc.array_empty_text,
                                                       desc.array_null_text);
            ImGui::TreePop();
            return changed;
        }

        return desc.draw_child_array("child_array_sheet",
                                     object,
                                     false,
                                     desc.array_empty_text,
                                     desc.array_null_text);
    }
    }

    return false;
}
#endif

template <typename TObject>
bool DrawPropertyWidget(TObject& object, const PropertyDesc<TObject>& desc)
{
#if defined(USE_IMGUI)
    return DrawPropertyWidgetImpl(object, desc, false);
#else
    (void)object;
    (void)desc;
    return false;
#endif
}

template <typename TObject, typename TFn>
bool ForEachProperty(TObject& object, TFn&& callback)
{
    const PropertyList<TObject>& list = GetPropertyList<TObject>();

    using ReturnType = std::invoke_result_t<TFn&, TObject&, const PropertyDesc<TObject>&>;
    bool changed = false;

    for (const PropertyDesc<TObject>& desc : list.items) {
        if constexpr (std::is_convertible<ReturnType, bool>::value) {
            changed = static_cast<bool>(callback(object, desc)) || changed;
        } else {
            callback(object, desc);
        }
    }

    return changed;
}

template <typename TObject>
bool DrawPropertyRow(TObject& object, const PropertyDesc<TObject>& desc, int tree_depth)
{
#if defined(USE_IMGUI)
    switch (desc.kind) {
    case PropertyKind::ChildObject: {
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);

        void* child_ptr = desc.get_child ? desc.get_child(object) : nullptr;
        if (!child_ptr) {
            ImGui::TreeNodeEx(desc.label ? desc.label : "",
                              ImGuiTreeNodeFlags_Leaf | ImGuiTreeNodeFlags_NoTreePushOnOpen);
            ImGui::TableSetColumnIndex(1);
            ImGui::TextUnformatted(desc.child_null_text ? desc.child_null_text : "<null>");
            return false;
        }

        const bool open = ImGui::TreeNodeEx(desc.label ? desc.label : "", ImGuiTreeNodeFlags_DefaultOpen);
        ImGui::TableSetColumnIndex(1);

        if (!open) {
            return false;
        }

        bool changed = false;
        if (desc.draw_child_rows) {
            changed = desc.draw_child_rows(child_ptr, tree_depth + 1) || changed;
        }
        ImGui::TreePop();
        return changed;
    }
    case PropertyKind::ChildArray: {
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        const bool open = ImGui::TreeNodeEx(desc.label ? desc.label : "");
        ImGui::TableSetColumnIndex(1);

        if (!open) {
            return false;
        }

        bool changed = false;
        if (desc.draw_child_array_rows) {
            changed = desc.draw_child_array_rows(object,
                                                 tree_depth + 1,
                                                 desc.array_empty_text,
                                                 desc.array_null_text) || changed;
        }
        ImGui::TreePop();
        return changed;
    }
    case PropertyKind::Section: {
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::TextUnformatted(desc.label ? desc.label : "");
        ImGui::Separator();
        ImGui::TableSetColumnIndex(1);
        return false;
    }
    default:
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::TextUnformatted(desc.label ? desc.label : "");
        ImGui::TableSetColumnIndex(1);
        ImGui::PushID(static_cast<const void*>(&desc));
        {
            const bool changed = DrawPropertyWidgetImpl(object, desc, true);
            ImGui::PopID();
            return changed;
        }
    }
#else
    (void)object;
    (void)desc;
    (void)tree_depth;
    return false;
#endif
}

//template <> struct PropertyBaseType<::MainShip> { using type = ::GameObject; };
//static_assert(!std::is_void<PropertyBaseType<MainShip>::type>::value, "PropertyBaseType<T>::type must not be void.");

template <typename TObject>
bool DrawPropertyRowsWithBases(TObject& object, int tree_depth)
{
#if defined(USE_IMGUI)
    bool changed = false;

    using BaseType = typename PropertyBaseType<TObject>::type;
    if constexpr (!std::is_void<BaseType>::value) {
        static_assert(std::is_base_of<BaseType, TObject>::value,
                      "PropertyBaseType<T>::type must be a base class of T.");
        if constexpr (HasPropertyList<BaseType>::value) {
            BaseType& base_ref = static_cast<BaseType&>(object);
            changed = DrawPropertyRowsWithBases(base_ref, tree_depth) || changed;
        }
    }

    const PropertyList<TObject>& list = GetPropertyList<TObject>();
    for (const PropertyDesc<TObject>& desc : list.items) {
        changed = DrawPropertyRow(object, desc, tree_depth) || changed;
    }
    return changed;
#else
    (void)object;
    (void)tree_depth;
    return false;
#endif
}

template <typename TObject>
bool DrawPropertySheetTable(const char* id, TObject& object)
{
#if defined(USE_IMGUI)
    const ImGuiTableFlags table_flags = //ImGuiTableFlags_BordersInnerV |
                                        ImGuiTableFlags_RowBg |
                                        ImGuiTableFlags_Resizable |
                                        //ImGuiTableFlags_SizingStretchProp | 
                                        ImGuiTableFlags_Borders;

    if (!ImGui::BeginTable(id, 2, table_flags)) {
        return false;
    }

    //ImGui::TableSetupColumn("Property", ImGuiTableColumnFlags_WidthStretch, 0.45f);
    //ImGui::TableSetupColumn("Value", ImGuiTableColumnFlags_WidthStretch, 0.55f);

    const bool changed = DrawPropertyRowsWithBases(object, 0);

    ImGui::EndTable();
    return changed;
#else
    (void)id;
    (void)object;
    return false;
#endif
}

template <typename TContainer, typename TLabelFn>
bool DrawPropertySheetFlatList(const char* id, TContainer& items, TLabelFn&& label_fn)
{
#if defined(USE_IMGUI)
    bool changed = false;
    bool first = true;
    size_t index = 0;

    ImGui::PushID(id);
    for (auto& item : items) {
        if (!first) {
            ImGui::Separator();
        }
        first = false;

        ImGui::PushID(static_cast<int>(index));
        const std::string label = detail::normalize_label(label_fn(item, index));
        if (!label.empty()) {
            ImGui::TextUnformatted(label.c_str());
        }
        changed = detail::draw_element_sheet("sheet", item) || changed;
        ImGui::PopID();

        ++index;
    }
    ImGui::PopID();

    return changed;
#else
    (void)id;
    (void)items;
    (void)label_fn;
    return false;
#endif
}

template <typename TContainer>
bool DrawPropertySheetFlatList(const char* id, TContainer& items)
{
    return DrawPropertySheetFlatList(id, items,
                                     [](const auto&, size_t index) {
                                         return std::string("Item ") + std::to_string(index);
                                     });
}

template <typename TContainer, typename TLabelFn>
bool DrawPropertySheetCollapsibleList(const char* id, TContainer& items, TLabelFn&& label_fn)
{
#if defined(USE_IMGUI)
    bool changed = false;
    size_t index = 0;

    ImGui::PushID(id);
    for (auto& item : items) {
        ImGui::PushID(static_cast<int>(index));

        std::string label = detail::normalize_label(label_fn(item, index));
        if (label.empty()) {
            label = std::string("Item ") + std::to_string(index);
        }

        const bool open = ImGui::TreeNodeEx("entry", ImGuiTreeNodeFlags_None, "%s", label.c_str());
        if (open) {
            changed = detail::draw_element_sheet("sheet", item) || changed;
            ImGui::TreePop();
        }

        ImGui::PopID();
        ++index;
    }
    ImGui::PopID();

    return changed;
#else
    (void)id;
    (void)items;
    (void)label_fn;
    return false;
#endif
}

template <typename TContainer>
bool DrawPropertySheetCollapsibleList(const char* id, TContainer& items)
{
    return DrawPropertySheetCollapsibleList(id, items,
                                            [](const auto& item, size_t index) {
                                                //return std::string("Item ") + std::to_string(index);
                                                std::string str_name;
#if 0
                                                const char* name = item->GetName();
                                                if(name && name[0]!='\0') {
                                                    str_name = name;
                                                } else {
                                                    str_name = "<Unk>";
                                                }
#endif

                                                const char* type_name = imgui_props::detail::resolve_element_type_name(item);
                                                if(type_name && type_name[0]!='\0') {
                                                    return str_name + " | " + std::string(type_name);
                                                } else {
                                                    return str_name + " | " + std::string("<Unk>");
                                                }
                                            });
}

} // namespace imgui_props

#define PROPERTY_SUPPORT(TYPE) friend const ::imgui_props::PropertyList<TYPE>& ::imgui_props::GetPropertyList();

#define PROPERTY_LIST_BEGIN(TYPE)                                                               \
namespace imgui_props {                                                                         \
template <> inline const ::imgui_props::PropertyList<TYPE>& GetPropertyList<TYPE>()            \
{                                                                                                \
    using _imgui_props_type = TYPE;                                                             \
    static const ::imgui_props::PropertyList<_imgui_props_type> s_property_list = []() {       \
        ::imgui_props::PropertyListBuilder<_imgui_props_type> _imgui_props_builder;             \
        _imgui_props_builder.TypeName(#TYPE);                                                    \

#define PROPERTY_LIST_BEGIN_DERIVED(TYPE, BASE_TYPE)                                            \
namespace imgui_props {                                                                         \
template <> inline const ::imgui_props::PropertyList<TYPE>& GetPropertyList<TYPE>()            \
{                                                                                                \
    using _imgui_props_type = TYPE;                                                             \
    static const ::imgui_props::PropertyList<_imgui_props_type> s_property_list = []() {       \
        ::imgui_props::PropertyListBuilder<_imgui_props_type> _imgui_props_builder;             \
        _imgui_props_builder.TypeName(#TYPE);                                                    \

#define PROPERTY_SECTION(LABEL) _imgui_props_builder.Section((LABEL))
#define PROPERTY_READONLY_TEXT(LABEL, ACCESSOR) _imgui_props_builder.ReadOnlyText((LABEL), (ACCESSOR))
#define PROPERTY_BOOL(MEMBER, LABEL) _imgui_props_builder.Bool((LABEL), &_imgui_props_type::MEMBER)
#define PROPERTY_INT(MEMBER, LABEL, ...) \
    _imgui_props_builder.Int((LABEL), &_imgui_props_type::MEMBER, ##__VA_ARGS__)
#define PROPERTY_UINT(MEMBER, LABEL, ...) \
    _imgui_props_builder.UInt((LABEL), &_imgui_props_type::MEMBER, ##__VA_ARGS__) 
#define PROPERTY_FLOAT(MEMBER, LABEL, ...) \
    _imgui_props_builder.Float((LABEL), &_imgui_props_type::MEMBER, ##__VA_ARGS__) // ## - is a GNU extention so remove preceding comma if zero args passed
#define PROPERTY_FLOAT_ACC(LABEL, ...) \
    _imgui_props_builder.Float((LABEL), nullptr, ##__VA_ARGS__) // ## - is a GNU extention so remove preceding comma if zero args passed
#define PROPERTY_VEC2(MEMBER, LABEL, ...) \
    _imgui_props_builder.Vec2((LABEL), &_imgui_props_type::MEMBER, ##__VA_ARGS__)
#define PROPERTY_VEC2_ACC(LABEL, ...) \
    _imgui_props_builder.Vec2((LABEL), nullptr, ##__VA_ARGS__)
#define PROPERTY_VEC3(MEMBER, LABEL, ...) \
    _imgui_props_builder.Vec3((LABEL), &_imgui_props_type::MEMBER, ##__VA_ARGS__)
#define PROPERTY_VEC3_RO(MEMBER, LABEL) \
    _imgui_props_builder.Vec3((LABEL), &_imgui_props_type::MEMBER, )
#define PROPERTY_VEC3_ACC(LABEL, ...) \
    _imgui_props_builder.Vec3((LABEL), nullptr, ##__VA_ARGS__)
#define PROPERTY_VEC4(MEMBER, LABEL, ...) \
    _imgui_props_builder.Vec4((LABEL), &_imgui_props_type::MEMBER, ##__VA_ARGS__)
#define PROPERTY_CHILD_OBJECT(MEMBER, LABEL) \
    _imgui_props_builder.Child<decltype(_imgui_props_type::MEMBER), &_imgui_props_type::MEMBER>((LABEL))
#define PROPERTY_CHILD_PTR(MEMBER, LABEL) \
    _imgui_props_builder.ChildPtr<typename std::remove_pointer<decltype(_imgui_props_type::MEMBER)>::type, &_imgui_props_type::MEMBER>((LABEL))
#define PROPERTY_ARRAY(MEMBER, LABEL) \
    _imgui_props_builder.Array<decltype(_imgui_props_type::MEMBER), &_imgui_props_type::MEMBER>((LABEL))
#define PROPERTY_ARRAY_FLAT(MEMBER, LABEL) \
    _imgui_props_builder.Array<decltype(_imgui_props_type::MEMBER), &_imgui_props_type::MEMBER>((LABEL), false)
#define PROPERTY_ARRAY_PTR(MEMBER, LABEL) \
    _imgui_props_builder.ArrayPtr<typename std::remove_pointer<decltype(_imgui_props_type::MEMBER)>::type, &_imgui_props_type::MEMBER>((LABEL))
#define PROPERTY_ARRAY_PTR_FLAT(MEMBER, LABEL) \
    _imgui_props_builder.ArrayPtr<typename std::remove_pointer<decltype(_imgui_props_type::MEMBER)>::type, &_imgui_props_type::MEMBER>((LABEL), false)

#define PROPERTY_LIST_END()                                                                     \
        return _imgui_props_builder.Build();                                                    \
    }();                                                                                        \
    return s_property_list;                                                                     \
}                                                                                               \
} // namespace imgui_props
  //
#define PROPERTY_LIST_DECLARE_DERIVED(TYPE, BASE_TYPE)                                           \
namespace imgui_props {\
    template <> struct HasPropertyList<TYPE> : std::true_type {};  \
    template <> struct PropertyBaseType<TYPE> { using type = BASE_TYPE; };                        \
}\
    template <> const ::imgui_props::PropertyList<TYPE>& ::imgui_props::GetPropertyList<TYPE>();\

#define PROPERTY_LIST_DECLARE(TYPE)                                                               \
namespace imgui_props {\
    template <> struct HasPropertyList<TYPE> : std::true_type {};  \
}\
    template <> const ::imgui_props::PropertyList<TYPE>& ::imgui_props::GetPropertyList<TYPE>();  

#define PROPERTY_POLYMORPHIC_DRAW_IMPL(TYPE)                                                      \
    bool DrawPolymorphicPropertySheet(const char* id) override                                    \
    {                                                                                              \
        return ::imgui_props::DrawPropertySheetTable(id, *static_cast<TYPE*>(this));              \
    }                                                                                              \
    bool DrawPolymorphicPropertyRows(int tree_depth) override                                      \
    {                                                                                              \
        return ::imgui_props::DrawPropertyRowsWithBases(*static_cast<TYPE*>(this), tree_depth);   \
    }                                                                                              \
    const char* GetPolymorphicPropertyTypeName() const override                                    \
    {                                                                                              \
        return #TYPE;                                                                              \
    }

#define PROPERTY_POLYMORPHIC_DRAW_DECL(TYPE)                                                      \
    bool DrawPolymorphicPropertySheet(const char* id) override;                                   \
    bool DrawPolymorphicPropertyRows(int tree_depth) override;                                    \
    const char* GetPolymorphicPropertyTypeName() const override

#define PROPERTY_POLYMORPHIC_DRAW_IMPL_EXT(TYPE)                                                  \
    bool TYPE::DrawPolymorphicPropertySheet(const char* id)                                       \
    {                                                                                              \
        return ::imgui_props::DrawPropertySheetTable(id, *static_cast<TYPE*>(this));              \
    }                                                                                              \
    bool TYPE::DrawPolymorphicPropertyRows(int tree_depth)                                         \
    {                                                                                              \
        return ::imgui_props::DrawPropertyRowsWithBases(*static_cast<TYPE*>(this), tree_depth);   \
    }                                                                                              \
    const char* TYPE::GetPolymorphicPropertyTypeName() const                                       \
    {                                                                                              \
        return #TYPE;                                                                              \
    }
