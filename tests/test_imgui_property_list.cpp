#include <cassert>
#include <list>
#include <string>
#include <vector>

#include "engine/utils/imgui_property_list.h"

struct Wheel {
    float radius = 0.35f;
    bool punctured = false;
};

PROPERTY_LIST_BEGIN(Wheel)
    PROPERTY_FLOAT(radius, "Radius", 0, 0.0f, 5.0f, 0.01f);
    PROPERTY_BOOL(punctured, "Punctured");
PROPERTY_LIST_END()

struct CarAttachment : public imgui_props::IPolymorphicPropertyObject {
    int mount_index = 0;

    virtual ~CarAttachment() = default;
    virtual const char* TypeName() const = 0;
};

PROPERTY_LIST_BEGIN(CarAttachment)
    PROPERTY_INT(mount_index, "Mount Index", 0, -1, 32, 1);
PROPERTY_LIST_END()

struct TurboAttachment : public CarAttachment {
    float boost = 1.0f;
    bool enabled = true;

    const char* TypeName() const override
    {
        return "Turbo";
    }

    PROPERTY_POLYMORPHIC_DRAW_IMPL(TurboAttachment)
};

PROPERTY_LIST_BEGIN_DERIVED(TurboAttachment, CarAttachment)
    PROPERTY_FLOAT(boost, "Boost", 0, 0.0f, 5.0f, 0.05f);
    PROPERTY_BOOL(enabled, "Enabled");
PROPERTY_LIST_END()

struct CargoAttachment : public CarAttachment {
    int capacity = 100;
    bool strapped = false;

    const char* TypeName() const override
    {
        return "Cargo";
    }

    PROPERTY_POLYMORPHIC_DRAW_IMPL(CargoAttachment)
};

PROPERTY_LIST_BEGIN_DERIVED(CargoAttachment, CarAttachment)
    PROPERTY_INT(capacity, "Capacity", 0, 2000, 10);
    PROPERTY_BOOL(strapped, "Strapped");
PROPERTY_LIST_END()

struct Car {
    vec3 velocity = vec3(0.0f, 0.0f, 0.0f);
    float mass = 1.0f;
    int gear = 0;
    bool engine_on = false;
    Wheel wheel;
    Wheel* spare_wheel = nullptr;
    std::list<Wheel> service_wheels;
    std::list<Wheel>* temporary_wheels = nullptr;
    std::list<CarAttachment*> attachments;
    std::string name;
};

PROPERTY_LIST_BEGIN(Car)
    PROPERTY_SECTION("Physics");
    PROPERTY_VEC3(velocity, "Velocity", 0.05f);
    PROPERTY_FLOAT(mass, "Mass", 0, 0.0f, 5000.0f, 0.1f);
    PROPERTY_CHILD_OBJECT(wheel, "Wheel");
    PROPERTY_CHILD_PTR(spare_wheel, "Spare Wheel");
    PROPERTY_ARRAY(service_wheels, "Service Wheels");
    PROPERTY_ARRAY_PTR(temporary_wheels, "Temporary Wheels");
    PROPERTY_ARRAY(attachments, "Attachments");
    PROPERTY_INT(gear, "Gear", 0, -1, 8);
    PROPERTY_BOOL(engine_on, "Engine On");
    PROPERTY_SECTION("Debug");
    PROPERTY_READONLY_TEXT("Name", [](const Car& car) { return car.name; });
PROPERTY_LIST_END()

template<typename T>
static const imgui_props::PropertyDesc<T>* find_property(const imgui_props::PropertyList<T>& list,
                                                           const char* label,
                                                           imgui_props::PropertyKind kind)
{
    for (const imgui_props::PropertyDesc<T>& desc : list.items) {
        if (std::string(desc.label ? desc.label : "") == std::string(label) && desc.kind == kind) {
            return &desc;
        }
    }
    return nullptr;
}

struct TestPropBase: public imgui_props::IPolymorphicPropertyObject {
  private:

	vec3 scale_;
	vec3 pos_;
	vec3 wpos_;

  public:
    PROPERTY_SUPPORT(TestPropBase);
    PROPERTY_POLYMORPHIC_DRAW_IMPL(TestPropBase);

	vec3 GetPosition() const {
		return wpos_;
	}
    void SetPosition(const vec3 &pos) {
        wpos_ = pos;
        pos_ = wpos_;
    }
};

PROPERTY_LIST_BEGIN_DERIVED(TestPropBase, void)
    PROPERTY_SECTION("TestPropBase");
    PROPERTY_VEC3_ACC("WorldPos", 0, 0.1f,
            [](const TestPropBase& c)->vec3{ return c.GetPosition();},
            [](TestPropBase& c, const vec3& pos){c.SetPosition(pos);});
    PROPERTY_VEC3(scale_, "Scale", 1);
PROPERTY_LIST_END()

struct TestPropDerived: public TestPropBase {
	std::string mesh_name_;
    const float size = 1.05f;
    float modifiable = 3.1415f;
    int32_t i32 = 10;
    const int32_t ri32 = 10;
    uint32_t u32 = 10;
    const uint32_t ru32 = 10;

    PROPERTY_SUPPORT(TestPropDerived)
    PROPERTY_POLYMORPHIC_DRAW_IMPL(TestPropDerived)
};

PROPERTY_LIST_BEGIN_DERIVED(TestPropDerived, TransformComponent)
    PROPERTY_SECTION("TestPropDerived");
    PROPERTY_FLOAT(size, "ro size");
    PROPERTY_FLOAT(modifiable, "modify me [-5, 10]", 0, -5.0f, 10.0f, 0.5f);
    PROPERTY_FLOAT_ACC("get(modifiable)", 0, -5.0f, 10.0f, 0.5f,
            [](const TestPropDerived& c)->float{ return c.modifiable;},
            [](TestPropDerived& c, float v){c.modifiable = v;});

    PROPERTY_INT(i32, "i32");
    PROPERTY_INT(ri32, "ri32");
    PROPERTY_INT(i32, "i32 as ro i32", PropertyFlags::kPropertyFlagReadOnly);
    PROPERTY_UINT(u32, "u32");
    PROPERTY_UINT(ru32, "ru32");
    PROPERTY_UINT(u32, "u32 as ro u32", PropertyFlags::kPropertyFlagReadOnly);
    PROPERTY_READONLY_TEXT("Name", [](const TestPropDerived& c){ return c.mesh_name_;});
PROPERTY_LIST_END()

static void test_registration_stability()
{
    const imgui_props::PropertyList<Car>& list_a = imgui_props::GetPropertyList<Car>();
    const imgui_props::PropertyList<Car>& list_b = imgui_props::GetPropertyList<Car>();
    const imgui_props::PropertyList<TurboAttachment>& turbo_list = imgui_props::GetPropertyList<TurboAttachment>();

    assert(&list_a == &list_b);
    assert(std::string(list_a.type_name) == "Car");
    assert(std::string(turbo_list.type_name) == "TurboAttachment");
    assert(list_a.items.size() == 12);

    assert(list_a.items[0].kind == imgui_props::PropertyKind::Section);
    assert(list_a.items[1].kind == imgui_props::PropertyKind::Vec3);
    assert(list_a.items[2].kind == imgui_props::PropertyKind::Float);
    assert(list_a.items[3].kind == imgui_props::PropertyKind::ChildObject);
    assert(list_a.items[4].kind == imgui_props::PropertyKind::ChildObject);
    assert(list_a.items[5].kind == imgui_props::PropertyKind::ChildArray);
    assert(list_a.items[6].kind == imgui_props::PropertyKind::ChildArray);
    assert(list_a.items[7].kind == imgui_props::PropertyKind::ChildArray);
    assert(list_a.items[8].kind == imgui_props::PropertyKind::Int);
    assert(list_a.items[9].kind == imgui_props::PropertyKind::Bool);
    assert(list_a.items[10].kind == imgui_props::PropertyKind::Section);
    assert(list_a.items[11].kind == imgui_props::PropertyKind::ReadOnlyText);
}

static void test_property_bindings()
{
    Car car;
    car.velocity = vec3(1.0f, 2.0f, 3.0f);
    car.mass = 123.0f;
    car.gear = 3;
    car.engine_on = false;
    car.wheel.radius = 0.42f;
    Wheel spare;
    spare.radius = 0.48f;
    car.spare_wheel = &spare;
    car.service_wheels.push_back(Wheel{0.5f, false});
    car.service_wheels.push_back(Wheel{0.6f, true});
    std::list<Wheel> temporary;
    temporary.push_back(Wheel{0.7f, false});
    car.temporary_wheels = &temporary;
    TurboAttachment turbo;
    turbo.boost = 1.4f;
    CargoAttachment cargo;
    cargo.capacity = 250;
    car.attachments.push_back(&turbo);
    car.attachments.push_back(&cargo);
    car.attachments.push_back(nullptr);
    car.name = "Interceptor";

    const imgui_props::PropertyList<Car>& list = imgui_props::GetPropertyList<Car>();

    const imgui_props::PropertyDesc<Car>* velocity_desc = find_property(list, "Velocity", imgui_props::PropertyKind::Vec3);
    assert(velocity_desc);
    assert(velocity_desc->vec3_member != nullptr);
    assert(velocity_desc->get_vec3 == nullptr);
    assert(velocity_desc->set_vec3 == nullptr);
    assert(car.*(velocity_desc->vec3_member) == vec3(1.0f, 2.0f, 3.0f));
    car.*(velocity_desc->vec3_member) = vec3(7.0f, 8.0f, 9.0f);
    assert(car.velocity == vec3(7.0f, 8.0f, 9.0f));

    const imgui_props::PropertyDesc<Car>* mass_desc = find_property(list, "Mass", imgui_props::PropertyKind::Float);
    assert(mass_desc);
    assert(mass_desc->float_member != nullptr);
    assert(mass_desc->get_float == nullptr);
    assert(mass_desc->set_float == nullptr);
    assert(mass_desc->meta.fminv == 0.0f);
    assert(mass_desc->meta.fmaxv == 5000.0f);
    car.*(mass_desc->float_member) = 150.0f;
    assert(car.mass == 150.0f);

    const imgui_props::PropertyDesc<Car>* wheel_desc = find_property(list, "Wheel", imgui_props::PropertyKind::ChildObject);
    assert(wheel_desc);
    assert(wheel_desc->get_child != nullptr);
    assert(wheel_desc->draw_child != nullptr);
    void* wheel_ptr = wheel_desc->get_child(car);
    assert(wheel_ptr == &car.wheel);
    Wheel* wheel = static_cast<Wheel*>(wheel_ptr);
    assert(wheel->radius == 0.42f);

    const imgui_props::PropertyDesc<Car>* spare_desc = find_property(list, "Spare Wheel", imgui_props::PropertyKind::ChildObject);
    assert(spare_desc);
    assert(spare_desc->get_child != nullptr);
    assert(spare_desc->draw_child != nullptr);
    void* spare_ptr = spare_desc->get_child(car);
    assert(spare_ptr == &spare);

    const imgui_props::PropertyDesc<Car>* service_desc = find_property(list, "Service Wheels", imgui_props::PropertyKind::ChildArray);
    assert(service_desc);
    assert(service_desc->draw_child_array != nullptr);
    assert(service_desc->array_collapsible == true);
    assert(service_desc->array_empty_text != nullptr);
    assert(service_desc->array_null_text != nullptr);

    const imgui_props::PropertyDesc<Car>* temporary_desc = find_property(list, "Temporary Wheels", imgui_props::PropertyKind::ChildArray);
    assert(temporary_desc);
    assert(temporary_desc->draw_child_array != nullptr);
    assert(temporary_desc->array_collapsible == true);
    assert(temporary_desc->array_empty_text != nullptr);
    assert(temporary_desc->array_null_text != nullptr);

    const imgui_props::PropertyDesc<Car>* attachments_desc = find_property(list, "Attachments", imgui_props::PropertyKind::ChildArray);
    assert(attachments_desc);
    assert(attachments_desc->draw_child_array != nullptr);
    assert(attachments_desc->array_collapsible == true);
    assert(attachments_desc->array_empty_text != nullptr);
    assert(attachments_desc->array_null_text != nullptr);

    const imgui_props::PropertyDesc<Car>* gear_desc = find_property(list, "Gear", imgui_props::PropertyKind::Int);
    assert(gear_desc);
    assert(gear_desc->int_member != nullptr);
    assert(gear_desc->get_int == nullptr);
    assert(gear_desc->set_int == nullptr);
    assert(gear_desc->meta.iminv == -1);
    assert(gear_desc->meta.imaxv == 8);
    car.*(gear_desc->int_member) = 5;
    assert(car.gear == 5);

    const imgui_props::PropertyDesc<Car>* engine_desc = find_property(list, "Engine On", imgui_props::PropertyKind::Bool);
    assert(engine_desc);
    assert(engine_desc->bool_member != nullptr);
    assert(engine_desc->get_bool == nullptr);
    assert(engine_desc->set_bool == nullptr);
    assert((car.*(engine_desc->bool_member)) == false);
    car.*(engine_desc->bool_member) = true;
    assert(car.engine_on == true);

    const imgui_props::PropertyDesc<Car>* name_desc = find_property(list, "Name", imgui_props::PropertyKind::ReadOnlyText);
    assert(name_desc && name_desc->get_text);
    assert(name_desc->get_text(car) == "Interceptor");
}

static void test_ro_properties() {
    TestPropDerived d;
    d.mesh_name_ = "MyMesh";

    const imgui_props::PropertyList<TestPropDerived>& list = imgui_props::GetPropertyList<TestPropDerived>();
    const imgui_props::PropertyDesc<TestPropDerived>* i32_desc = find_property(list, "i32", imgui_props::PropertyKind::Int);
    assert(i32_desc->flags == 0);
    const imgui_props::PropertyDesc<TestPropDerived>* ri32_desc = find_property(list, "ri32", imgui_props::PropertyKind::Int);
    assert(ri32_desc->flags == imgui_props::PropertyFlags::kPropertyFlagReadOnly);
    const imgui_props::PropertyDesc<TestPropDerived>* i32_as_ri32_desc = find_property(list, "i32 as ro i32", imgui_props::PropertyKind::Int);
    assert(i32_as_ri32_desc->flags == imgui_props::PropertyFlags::kPropertyFlagReadOnly);
}


static void test_foreach_custom_layout_flow()
{
    Car car;
    car.mass = 100.0f;

    std::vector<std::string> labels;
    imgui_props::ForEachProperty(car,
                                 [&labels](Car&, const imgui_props::PropertyDesc<Car>& desc) {
                                     labels.push_back(desc.label ? desc.label : "");
                                 });

    assert(labels.size() == imgui_props::GetPropertyList<Car>().items.size());
    assert(labels[0] == "Physics");
    assert(labels[1] == "Velocity");
    assert(labels[2] == "Mass");

    bool changed = imgui_props::ForEachProperty(car,
                                                [](Car& entry, const imgui_props::PropertyDesc<Car>& desc) -> bool {
                                                    if (desc.kind == imgui_props::PropertyKind::Float) {
                                                        if (desc.set_float) {
                                                            desc.set_float(entry, 250.0f);
                                                            return true;
                                                        }
                                                        if (desc.float_member) {
                                                            entry.*(desc.float_member) = 250.0f;
                                                            return true;
                                                        }
                                                    }
                                                    return false;
                                                });
    assert(changed);
    assert(car.mass == 250.0f);
}

static void test_vector_layout_compile_paths()
{
    std::vector<Car> cars;
    cars.resize(2);
    cars[0].name = "Alpha";
    cars[1].name = "Beta";

    auto label_fn = [](const Car& car, size_t index) -> std::string {
        return car.name + " [" + std::to_string(index) + "]";
    };

#if defined(USE_IMGUI)
    if (false) {
        imgui_props::DrawPropertySheetFlatList("cars_flat", cars, label_fn);
        imgui_props::DrawPropertySheetCollapsibleList("cars_tree", cars, label_fn);
        imgui_props::DrawPropertySheetTable("car_single", cars[0]);

        const imgui_props::PropertyList<Car>& list = imgui_props::GetPropertyList<Car>();
        imgui_props::DrawPropertyWidget(cars[0], list.items[1]);
    }
#endif

    (void)label_fn;
}

static void test_polymorphic_array_compile_paths()
{
    TurboAttachment turbo;
    turbo.boost = 1.8f;

    CargoAttachment cargo;
    cargo.capacity = 320;

    std::list<CarAttachment*> attachments;
    attachments.push_back(&turbo);
    attachments.push_back(&cargo);
    attachments.push_back(nullptr);

    auto label_fn = [](CarAttachment* attachment, size_t index) -> std::string {
        if (!attachment) {
            return std::string("<null> [") + std::to_string(index) + "]";
        }
        return std::string(attachment->TypeName()) + " [" + std::to_string(index) + "]";
    };

#if defined(USE_IMGUI)
    if (false) {
        imgui_props::DrawPropertySheetFlatList("attachments_flat", attachments, label_fn);
        imgui_props::DrawPropertySheetCollapsibleList("attachments_tree", attachments, label_fn);
    }
#endif

    (void)label_fn;
}

void test_imgui_property_list()
{
    test_registration_stability();
    test_property_bindings();
    test_ro_properties();
    test_foreach_custom_layout_flow();
    test_vector_layout_compile_paths();
    test_polymorphic_array_compile_paths();
}
