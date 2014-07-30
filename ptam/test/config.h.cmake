#ifndef CONFIG_H
#define CONFIG_H

#cmakedefine DATA_DIR "@DATA_DIR@"

#define DATA_PATH(file) DATA_DIR "/" file

#define CONFIG_FILE  "${DATA_DIR}/test.ini"

#undef BOOST_NO_INCLASS_MEMBER_INITIALIZATION

#endif // CONFIG_H
