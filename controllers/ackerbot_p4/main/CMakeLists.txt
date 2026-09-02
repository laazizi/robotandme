# Un controleur = robot.h (ici) + le code commun + UNE cinematique.
# La cinematique se choisit par le dossier liste ci-dessous, rien d'autre.
set(COMMON "${CMAKE_CURRENT_LIST_DIR}/../../common")

idf_component_register(
    SRC_DIRS     "${COMMON}/base" "${COMMON}/kin_ackermann"
    INCLUDE_DIRS "." "${COMMON}/base" "${COMMON}/kin_ackermann"
    PRIV_REQUIRES driver micro_ros_espidf_component
)
