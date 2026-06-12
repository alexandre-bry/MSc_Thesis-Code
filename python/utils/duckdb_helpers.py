import logging
from pathlib import Path
from typing import Optional

import duckdb


def connect_to_duckdb(db_file: Optional[Path] = None) -> duckdb.DuckDBPyConnection:
    con = duckdb.connect(
        database=":memory:" if db_file is None else str(db_file),
        read_only=False,
        config={"storage_compatibility_version": "v1.5.0"},
    )
    con.sql("INSTALL spatial; LOAD spatial;")
    return con


def create_schema(con: duckdb.DuckDBPyConnection, schema_name: str):
    con.execute(f"CREATE SCHEMA IF NOT EXISTS {schema_name};")


def export_parquet(
    con: duckdb.DuckDBPyConnection,
    table_name: str,
    geom_col_name: str,
    output_file: Path,
):
    # Get the columns of the table
    columns = con.execute(f"""
        SELECT column_name
        FROM information_schema.columns
        WHERE table_name = '{table_name.split('.')[-1]}'
            AND table_schema = '{table_name.split('.')[0]}';
    """).fetchall()

    if not columns:
        raise ValueError(f"Could not retrieve columns for table '{table_name}'")

    columns = [col[0] for col in columns]
    if "bbox" not in columns:
        bbox_column = f"""
            STRUCT_PACK(
                xmin := ST_XMin("{geom_col_name}"),
                ymin := ST_YMin("{geom_col_name}"),
                xmax := ST_XMax("{geom_col_name}"),
                ymax := ST_YMax("{geom_col_name}")
            ) AS bbox
            """
        columns.append(bbox_column)

    # Calculate dataset bounds
    bounds_result = con.execute(f"""
        SELECT
            MIN(ST_XMin("{geom_col_name}")) as xmin,
            MIN(ST_YMin("{geom_col_name}")) as ymin,
            MAX(ST_XMax("{geom_col_name}")) as xmax,
            MAX(ST_YMax("{geom_col_name}")) as ymax
        FROM {table_name};
    """).fetchone()

    if not bounds_result or any(v is None for v in bounds_result):
        raise ValueError("Could not calculate dataset bounds from table")

    xmin, ymin, xmax, ymax = bounds_result

    # Export the table to Parquet, ordered by Hilbert curve for spatial locality
    con.execute(
        f"""
        COPY (
            SELECT {', '.join(columns)} FROM {table_name}
            ORDER BY ST_Hilbert(
                "{geom_col_name}",
                ST_Extent(ST_MakeEnvelope({xmin}, {ymin}, {xmax}, {ymax}))
            )
        ) TO $output_file
        (FORMAT parquet, COMPRESSION zstd, ROW_GROUP_SIZE 100_000);
        """,
        {"output_file": str(output_file)},
    )


class DuckDBConnector:
    def __init__(self, db_file: Optional[Path] = None):
        self.db_file = db_file
        self.con = connect_to_duckdb(self.db_file)

    def execute(self, query: str, params: Optional[dict] = None):
        if self.con is None:
            raise ValueError("DuckDB connection is not established")

        # Get the query string with the parameters substituted for logging
        query_str = query
        if params:
            for key, value in params.items():
                query_str = query_str.replace(f"${key}", f"'{value}'")
        logging.debug(f"Executing query:\n{query_str}")
        return self.con.execute(query, params)

    def create_schema(self, schema_name: str):
        self.execute(f"CREATE SCHEMA IF NOT EXISTS {schema_name};")

    def export_parquet(
        self, schema_name: str, table_name: str, geom_col_name: str, output_file: Path
    ):
        # Get the columns of the table
        columns = self.execute(f"""
            SELECT column_name
            FROM information_schema.columns
            WHERE table_name = '{table_name}'
                AND table_schema = '{schema_name}';
        """).fetchall()

        if not columns:
            raise ValueError(
                f"Could not retrieve columns for table '{schema_name}.{table_name}'"
            )

        columns = [col[0] for col in columns]
        if "bbox" not in columns:
            bbox_column = f"""
                STRUCT_PACK(
                    xmin := ST_XMin("{geom_col_name}"),
                    ymin := ST_YMin("{geom_col_name}"),
                    xmax := ST_XMax("{geom_col_name}"),
                    ymax := ST_YMax("{geom_col_name}")
                ) AS bbox
                """
            columns.append(bbox_column)

        # Calculate dataset bounds
        bounds_result = self.execute(f"""
            SELECT
                MIN(ST_XMin("{geom_col_name}")) as xmin,
                MIN(ST_YMin("{geom_col_name}")) as ymin,
                MAX(ST_XMax("{geom_col_name}")) as xmax,
                MAX(ST_YMax("{geom_col_name}")) as ymax
            FROM {schema_name}.{table_name};
        """).fetchone()

        if not bounds_result or any(v is None for v in bounds_result):
            raise ValueError("Could not calculate dataset bounds from table")

        xmin, ymin, xmax, ymax = bounds_result

        # Export the table to Parquet, ordered by Hilbert curve for spatial locality
        self.execute(
            f"""
            COPY (
                SELECT {', '.join(columns)} FROM {schema_name}.{table_name}
                ORDER BY ST_Hilbert(
                    "{geom_col_name}",
                    ST_Extent(ST_MakeEnvelope({xmin}, {ymin}, {xmax}, {ymax}))
                )
            ) TO $output_file
            (FORMAT parquet, COMPRESSION zstd, ROW_GROUP_SIZE 100_000);
            """,
            {"output_file": str(output_file)},
        )

    def close(self):
        self.con.close()
        if self.db_file is not None and self.db_file.exists():
            self.db_file.unlink()


class DuckDBConnectionManager:
    def __init__(self, db_file: Optional[Path] = None):
        self.db_file = db_file
        self.con: Optional[DuckDBConnector] = None

    def __enter__(self) -> DuckDBConnector:
        self.con = DuckDBConnector(self.db_file)
        return self.con

    def __exit__(self, exc_type, exc_val, exc_tb):
        if self.con is not None:
            self.con.close()
