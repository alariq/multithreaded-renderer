#define DO_TESTS

void tests_run() {
#if defined(DO_TESTS)
    //extern void test_fixed_block_allocator();
    //test_fixed_block_allocator();
    extern void test_myarray();
    test_myarray();
    extern void test_spline();
    test_spline();
#if defined(USE_IMGUI)
    extern void test_imgui_property_list();
    test_imgui_property_list();
#endif
}
#else
void tests_run() {}
#endif

