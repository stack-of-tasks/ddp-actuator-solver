SET(${PROJECT_NAME}_HEADERS
		awas.h
        costRomeoPos.h
		costRomeoTorque.h
		costVIA.h
        romeolinearactuator.h
        config.h
        dynamicmodel.h
        romeosimpleactuator.h
        costfunction.h
        ilqrsolver.h
		lqrsolver.h
        types.h
		dcmotor.h
        costDC.h
)

INSTALL(FILES ${${PROJECT_NAME}_HEADER} DESTINATION include)

