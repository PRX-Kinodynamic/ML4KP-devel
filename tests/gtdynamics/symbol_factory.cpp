#define BOOST_AUTO_TEST_MAIN symbols_factory_test
#include <string>
#include <boost/test/unit_test.hpp>
#include "prx/gtdynamics/defs.hpp"

BOOST_AUTO_TEST_CASE( symbols_factory_test )
{
	std::vector<std::string> symbol_names = 
		{"goal_symbol", "state_symbol", "control_symbol", "time_symbol"};

    // std::vector<std::pair<std::string, std::string>> fn_names = 
    //     {{"2D_Point", "2D_Point"}, {"rally_car", "rally_car"}, {"treaded_vehicle", "treaded_vehicle"}, {"FO_treaded_vehicle", "FO_treaded_vehicle"}};

    auto available_symbols = prx::symbol_factory_t::available_symbols();
    for (auto name : symbol_names)
    {
        printf("\tChecking available system: %s...", name.c_str() );
        BOOST_CHECK(std::find(available_symbols.begin(), available_symbols.end(), name) != available_symbols.end());
        printf("\t[ OK ]\n" );
    }

    printf("Checking symbols in Factory...\n");
    {
        printf("\tChecking symbol: %s...", symbol_names[0].c_str() );
        auto symbol = prx::symbol_factory_t::create_symbol(symbol_names[0], 1);
        // BOOST_CHECK( sys_ptr != nullptr ); 
        BOOST_CHECK( symbol.label() == "Xg"); 
        printf("\t[ OK ]\n" );
    }

    {
        printf("\tChecking symbol: %s...", symbol_names[1].c_str() );
        uint64_t ti = 10;
        auto symbol = prx::symbol_factory_t::create_symbol(symbol_names[1], ti);
        BOOST_CHECK( symbol.label() == "Xi"); 
        BOOST_CHECK( symbol.time() == ti); 
        printf("\t[ OK ]\n" );
    }

    {
        printf("\tChecking symbol: %s...", symbol_names[2].c_str() );
        uint64_t ti = 100;
        auto symbol = prx::symbol_factory_t::create_symbol(symbol_names[2], ti);
        BOOST_CHECK( symbol.label() == "Ui"); 
        BOOST_CHECK( symbol.time() == ti); 
        printf("\t[ OK ]\n" );
    }

    {
        printf("\tChecking symbol: %s...", symbol_names[3].c_str() );
        uint64_t ti = 50;
        auto symbol = prx::symbol_factory_t::create_symbol(symbol_names[3], ti);
        BOOST_CHECK( symbol.label() == "ti"); 
        BOOST_CHECK( symbol.time() == ti); 
        printf("\t[ OK ]\n" );
    }

}
