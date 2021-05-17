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

    void write_buffered_data(XSENS_POSITION_DATA positionData[], int numBufferedItems);
    void init_database_connection();
    void close_database_connection();
};

KinopedDB *xsens_data;

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
    char buffer[1024];
    int lib_ver = PQlibVersion();
	int ret;
    
    printf("Version of libpq: %d\n", lib_ver);
    memset(&buffer, 0, sizeof(buffer));
    sprintf(buffer, "createdb -h localhost -p 5432 -U %s  %s", dbUser, dbName);
    ret = system(buffer);

    sprintf(buffer, "host=localhost port=5432 user=%s dbname=%s", dbUser, dbName);
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
    printf("DB Name = %s\n", dbName);
    PQfinish(conn);
}

/*************************************************************************************************/
/*                                                                                               */
/* void KinopedDB::write_buffered_data(XSENS_POSITION_DATA positionData[], int numBufferedItems) */
/*                                                                                               */
/* Write the buffered data to the database. This is done as a number of rows to speed up writes. */
/*                                                                                               */
/*************************************************************************************************/
void KinopedDB::write_buffered_data(XSENS_POSITION_DATA positionData[], int numBufferedItems)
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
        sprintf(tempBuffer, rowInsertString, 
               rowID++,
               positionData[i].timeString,
               positionData[i].leftHipX,
               positionData[i].leftHipY,
               positionData[i].leftHipZ,
               positionData[i].leftKneeX,
               positionData[i].leftKneeY,
               positionData[i].leftKneeZ,
               positionData[i].leftFootX,
               positionData[i].leftFootY,
               positionData[i].leftFootZ,
               positionData[i].rightHipX,
               positionData[i].rightHipY,
               positionData[i].rightHipZ,
               positionData[i].rightKneeX,
               positionData[i].rightKneeY,
               positionData[i].rightKneeZ,
               positionData[i].rightFootX,
               positionData[i].rightFootY,
               positionData[i].rightFootZ,
               positionData[i].pulse,
               positionData[i].respirationRate,
               positionData[i].pulseOx,
               separator);
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
#if defined(DEBUG_TIMING_PRINTF)
    printf("Elapsed Postgres DB \"%s\" table \"%s\" write time %0.3lfms\n", dbName, dbTableName, (double)(ts2.tv_nsec - ts1.tv_nsec) / 1000000.0);
#endif
    if ((ts2.tv_nsec - ts1.tv_nsec) > maxVal) maxVal = (double)(ts2.tv_nsec - ts1.tv_nsec);
    if ((ts2.tv_nsec - ts1.tv_nsec) < minVal) minVal = (double)(ts2.tv_nsec - ts1.tv_nsec);
    totalTime += (double)(ts2.tv_nsec - ts1.tv_nsec);
        
    if (PQresultStatus(res) != PGRES_COMMAND_OK)
    {
        printf("write_buffered_data - write to DB \"%s\" table \"%s\" failed %d\n", dbName, dbTableName, PQresultStatus(res));
        do_exit(conn, res);
    }
    
    PQclear(res);
}

