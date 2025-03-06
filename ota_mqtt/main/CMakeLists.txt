set(COMPONENT_SRCS 
	"app_main.c"
	"ota_demo_core_mqtt.c"
	"mqtt_subscription_manager.c"
	)

set(COMPONENT_ADD_INCLUDEDIRS
	"."
	"${PROJECT_DIR}/../../../esp/v5.3.2/esp-idf/libraries/common/logging/"
	"${PROJECT_DIR}/../../../esp/v5.3.2/esp-idf/libraries/common/posix_compat/"
	)

idf_component_register(SRCS "${COMPONENT_SRCS}"
					   INCLUDE_DIRS ${COMPONENT_ADD_INCLUDEDIRS}
					  )