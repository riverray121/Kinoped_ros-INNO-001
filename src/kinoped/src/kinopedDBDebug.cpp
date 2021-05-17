class KinopedDB {
  private:
      PGconn *conn;
      PGresult *res;
      uint64_t rowID = 1;
      char dbName[MAX_DATABASE_NAME_LENGTH];
      char dbUser[MAX_USER_NAME_LENGTH];
      char dbTableName[MAX_DATABASE_TABLE_NAME];
      char dropTableString[MAX_INDIVIDUAL_VALUES_STRING_LENGTH];
      char createTableString[MAX_INDIVIDUAL_VALUES_STRING_LENGTH];
      char rowInsertString[MAX_INDIVIDUAL_VALUES_STRING_LENGTH];
      char bigInsertString[MAX_DB_INSERT_STRING_LENGTH];

  public:
    KinopedDB(char *db_name, char *user, char *db_table_name, char *drop_table_string, char *create_table_string, char *insert_string)
    {
        strcpy(dbName, db_name);
        strcpy(dbUser, user);
        strcpy(rowInsertString, insert_string);
        strcpy(createTableString, create_table_string);
        strcpy(dropTableString, drop_table_string);
        strcpy(dbTableName, db_table_name);
    }


    void write_buffered_data(PATIENT_ROW patientData[], int numBufferedItems);
    void init_database_connection();
    void close_database_connection();
};

/*******************************************************************************************/
/*                                                                                         */
/* void do_exit(PGconn *conn, PGresult *res)                                               */
/*                                                                                         */
/* Database error handler callback. Close DB connection and exit.                          */
/*                                                                                         */
/*******************************************************************************************/
void do_exit(PGconn *conn, PGresult *res)
{
    fprintf(stderr, "%s\n", PQerrorMessage(conn));    

    if (res != NULL)
    {
        PQclear(res);
        res = NULL;
    }
    if (conn != NULL)
    {
        PQfinish(conn);
        conn = NULL;
    }
    exit(1);
}


/*******************************************************************************************/
/*                                                                                         */
/* void KinopedDB::init_database_connection()                                              */
/*                                                                                         */
/* Create the database, open a connection to it, and create the necessary table.           */
/*                                                                                         */
/*******************************************************************************************/
void KinopedDB::init_database_connection()
{
    char timeString[MAX_TIMESTAMP_LENGTH];
    char newDBName[MAX_DATABASE_NAME_LENGTH];
    char buffer[1024];
    time_t t = time(NULL);
    clock_gettime(CLOCK_REALTIME, &ts1);
    struct tm tm = *localtime(&t);
    int lib_ver = PQlibVersion();
    
    memset(&newDBName, 0, sizeof(newDBName));
    strcpy(newDBName, dbName);
//    sprintf(newDBName, "%%04d_%02d_%02d_%02d_%02d_%02d", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
    strcat(newDBName, "_debug");

    printf("Version of libpq: %d\n", lib_ver);
    memset(&buffer, 0, sizeof(buffer));
    sprintf(buffer, "createdb -h localhost -p 5432 -U %s  %s", dbUser, newDBName);
    system(buffer);

    sprintf(buffer, "host=localhost port=5432 user=%s dbname=%s", dbUser, newDBName);
    conn = PQconnectdb(buffer);
    if (PQstatus(conn) == CONNECTION_BAD)
    {
        printf("Connection to database failed: %s\n", PQerrorMessage(conn));
        PQfinish(conn);
        exit(1);
    }
    else
    {
        printf("Connection to database OK\n");
    }

    res = PQexec(conn, createTableString);
        
    if (PQresultStatus(res) != PGRES_COMMAND_OK)
    {
        printf("Error creating table %s\n", createTableString);
        do_exit(conn, res);
    }
    
    PQclear(res);
}

/*******************************************************************************************/
/*                                                                                         */
/* void KinopedDB::close_database_connection()                                             */
/*                                                                                         */
/* Close the connection to the database.                                                   */
/*                                                                                         */
/*******************************************************************************************/
void KinopedDB::close_database_connection()
{
    PQfinish(conn);
}

/*******************************************************************************************/
/*                                                                                         */
/* void KinopedDB::write_buffered_data(PATIENT_ROW patientData[], int numBufferedItems)    */
/*                                                                                         */
/* Write the buffered data to the database. This is done as a number of rows to speed up   */
/*     writes.                                                                             */
/*                                                                                         */
/*******************************************************************************************/
void KinopedDB::write_buffered_data(PATIENT_ROW patientData[], int numBufferedItems)
{
    char tempBuffer[MAX_INDIVIDUAL_VALUES_STRING_LENGTH];
    char separator[4];
    
    memset(tempBuffer, 0, sizeof(tempBuffer));
    memset(bigInsertString, 0, sizeof(bigInsertString));
    strcpy(separator, ", ");

    sprintf(bigInsertString, "INSERT INTO %s VALUES ", dbTableName);
    for (int i = 0; i < numBufferedItems; i++)
    {
        memset(tempBuffer, 0, sizeof(tempBuffer));
        if (i == (numBufferedItems - 1))
        {
            strcpy(separator, ";");
        }
#if (0)
#ifdef INCLUDE_ALL_DATA_IN_DB
        sprintf(tempBuffer, "(%ld,'%s','%s','%s','%s',%d,%d,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf)%s", patientFirstName, patientMiddleName, patientLastName,
#else
        sprintf(tempBuffer, "(%ld,'%s',%d,%d,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf)%s",
#endif
#endif
        sprintf(tempBuffer, rowInsertString, 
               rowID++,
               patientData[i].timeString,
               patientData[i].dataSource, patientData[i].command, 
               patientData[i].joints[AR_JOINT_INDEX].current_value,
               patientData[i].joints[BR1_JOINT_INDEX].current_value,
               patientData[i].joints[CR1_JOINT_INDEX].current_value,
#ifdef INCLUDE_ALL_DATA_IN_DB
               patientData[i].joints[DR_JOINT_INDEX].current_value,
#endif
               patientData[i].joints[ER1_JOINT_INDEX].current_value,
               patientData[i].joints[FR1_JOINT_INDEX].current_value,
#ifdef INCLUDE_ALL_DATA_IN_DB
               patientData[i].joints[GR_JOINT_INDEX].current_value,
#endif
               patientData[i].joints[HR_JOINT_INDEX].current_value,
               patientData[i].joints[IR_JOINT_INDEX].current_value,
#ifdef INCLUDE_ALL_DATA_IN_DB
               patientData[i].joints[ER2_JOINT_INDEX].current_value,
               patientData[i].joints[FR2_JOINT_INDEX].current_value,
               patientData[i].joints[BR2_JOINT_INDEX].current_value,
               patientData[i].joints[CR2_JOINT_INDEX].current_value,
#endif
               patientData[i].joints[AL_JOINT_INDEX].current_value,
               patientData[i].joints[BL1_JOINT_INDEX].current_value,
               patientData[i].joints[CL1_JOINT_INDEX].current_value,
#ifdef INCLUDE_ALL_DATA_IN_DB
               patientData[i].joints[DL_JOINT_INDEX].current_value,
#endif
               patientData[i].joints[EL1_JOINT_INDEX].current_value,
               patientData[i].joints[FL1_JOINT_INDEX].current_value,
#ifdef INCLUDE_ALL_DATA_IN_DB
               patientData[i].joints[GL_JOINT_INDEX].current_value,
#endif
               patientData[i].joints[HL_JOINT_INDEX].current_value,
               patientData[i].joints[IL_JOINT_INDEX].current_value,
#ifdef INCLUDE_ALL_DATA_IN_DB
               patientData[i].joints[EL2_JOINT_INDEX].current_value,
               patientData[i].joints[FL2_JOINT_INDEX].current_value,
               patientData[i].joints[BL2_JOINT_INDEX].current_value,
               patientData[i].joints[CL2_JOINT_INDEX].current_value,
#endif
               patientData[i].joints[AR_JOINT_INDEX].velocity,
               patientData[i].joints[BR1_JOINT_INDEX].velocity,
               patientData[i].joints[CR1_JOINT_INDEX].velocity,
#ifdef INCLUDE_ALL_DATA_IN_DB
               patientData[i].joints[DR_JOINT_INDEX].velocity,
#endif
               patientData[i].joints[ER1_JOINT_INDEX].velocity,
               patientData[i].joints[FR1_JOINT_INDEX].velocity,
#ifdef INCLUDE_ALL_DATA_IN_DB
               patientData[i].joints[GR_JOINT_INDEX].velocity,
#endif
               patientData[i].joints[HR_JOINT_INDEX].velocity,
               patientData[i].joints[IR_JOINT_INDEX].velocity,
#ifdef INCLUDE_ALL_DATA_IN_DB
               patientData[i].joints[ER2_JOINT_INDEX].velocity,
               patientData[i].joints[FR2_JOINT_INDEX].velocity,
               patientData[i].joints[BR2_JOINT_INDEX].velocity,
               patientData[i].joints[CR2_JOINT_INDEX].velocity,
#endif
               patientData[i].joints[AL_JOINT_INDEX].velocity,
               patientData[i].joints[BL1_JOINT_INDEX].velocity,
               patientData[i].joints[CL1_JOINT_INDEX].velocity,
#ifdef INCLUDE_ALL_DATA_IN_DB
               patientData[i].joints[DL_JOINT_INDEX].velocity,
#endif
               patientData[i].joints[EL1_JOINT_INDEX].velocity,
               patientData[i].joints[FL1_JOINT_INDEX].velocity,
#ifdef INCLUDE_ALL_DATA_IN_DB
               patientData[i].joints[GL_JOINT_INDEX].velocity,
#endif
               patientData[i].joints[HL_JOINT_INDEX].velocity,
               patientData[i].joints[IL_JOINT_INDEX].velocity,
#ifdef INCLUDE_ALL_DATA_IN_DB
               patientData[i].joints[EL2_JOINT_INDEX].velocity,
               patientData[i].joints[FL2_JOINT_INDEX].velocity,
               patientData[i].joints[BL2_JOINT_INDEX].velocity,
               patientData[i].joints[CL2_JOINT_INDEX].velocity,
#endif
               patientData[i].joints[AR_JOINT_INDEX].effort,
               patientData[i].joints[BR1_JOINT_INDEX].effort,
               patientData[i].joints[CR1_JOINT_INDEX].effort,
#ifdef INCLUDE_ALL_DATA_IN_DB
               patientData[i].joints[DR_JOINT_INDEX].effort,
#endif
               patientData[i].joints[ER1_JOINT_INDEX].effort,
               patientData[i].joints[FR1_JOINT_INDEX].effort,
#ifdef INCLUDE_ALL_DATA_IN_DB
               patientData[i].joints[GR_JOINT_INDEX].effort,
#endif
               patientData[i].joints[HR_JOINT_INDEX].effort,
               patientData[i].joints[IR_JOINT_INDEX].effort,
#ifdef INCLUDE_ALL_DATA_IN_DB
               patientData[i].joints[ER2_JOINT_INDEX].effort,
               patientData[i].joints[FR2_JOINT_INDEX].effort,
               patientData[i].joints[BR2_JOINT_INDEX].effort,
               patientData[i].joints[CR2_JOINT_INDEX].effort,
#endif
               patientData[i].joints[AL_JOINT_INDEX].effort,
               patientData[i].joints[BL1_JOINT_INDEX].effort,
               patientData[i].joints[CL1_JOINT_INDEX].effort,
#ifdef INCLUDE_ALL_DATA_IN_DB
               patientData[i].joints[DL_JOINT_INDEX].effort,
#endif
               patientData[i].joints[EL1_JOINT_INDEX].effort,
               patientData[i].joints[FL1_JOINT_INDEX].effort,
#ifdef INCLUDE_ALL_DATA_IN_DB
               patientData[i].joints[GL_JOINT_INDEX].effort,
#endif
               patientData[i].joints[HL_JOINT_INDEX].effort,
               patientData[i].joints[IL_JOINT_INDEX].effort,
#ifdef INCLUDE_ALL_DATA_IN_DB
               patientData[i].joints[EL2_JOINT_INDEX].effort,
               patientData[i].joints[FL2_JOINT_INDEX].effort,
               patientData[i].joints[BL2_JOINT_INDEX].effort,
               patientData[i].joints[CL2_JOINT_INDEX].effort,
#endif
               separator);
printf("Insert string = %s\n", tempBuffer);
        strcat(bigInsertString, tempBuffer); 
    }

    clock_gettime(CLOCK_REALTIME, &ts1);
    res = PQexec(conn, bigInsertString);
    clock_gettime(CLOCK_REALTIME, &ts2);
    if (ts2.tv_nsec < ts1.tv_nsec)
    {
        ts2.tv_nsec += 1000000000;
        ts2.tv_sec--;
    }
//       printf("Elapsed Postgres DB write time %0.3lfms\n", (double)(ts2.tv_nsec - ts1.tv_nsec) / 1000000.0);
    if ((ts2.tv_nsec - ts1.tv_nsec) > maxVal) maxVal = (double)(ts2.tv_nsec - ts1.tv_nsec);
    if ((ts2.tv_nsec - ts1.tv_nsec) < minVal) minVal = (double)(ts2.tv_nsec - ts1.tv_nsec);
    totalTime += (double)(ts2.tv_nsec - ts1.tv_nsec);
        
    if (PQresultStatus(res) != PGRES_COMMAND_OK)
    {
        printf("write_patient_data - write to DB failed %d\n", PQresultStatus(res));
        do_exit(conn, res);
    }
    
    PQclear(res);
}

