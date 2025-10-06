#!/usr/bin/env node

import { Command } from "commander";
import { Ajv2020 } from "ajv/dist/2020";
import { readFile } from "fs/promises";
import { join, resolve } from "path";

const program = new Command();
const ajv = new Ajv2020({
	discriminator: true,
	verbose: true,
});

type IMU =
	| {
			protocol: "SPI";
			imu: string;
			int: number;
			rotation: string;
	  }
	| {
			protocol: "I2C";
			imu: string;
			int: number;
			sda: string;
			scl: string;
			rotation: string;
			address: number;
	  };
type Battery =
	| {
			type: "BAT_EXTERNAL";
			shieldR: number;
			r1: number;
			r2: number;
			pin: string;
	  }
	| {
			type: "BAT_INTERNAL" | "BAT_MCP3021" | "BAT_INTERNAL_MCP3021";
	  };
type Defaults = {
	defaults: Record<
		string,
		{
			values: {
				BOARD: string;
				LED_PIN?: string;
				LED_INVERTED?: boolean;
				SENSORS?: Partial<IMU>[];
				BATTERY?: Partial<Battery>;
			};
		}
	>;
};

const buildBoard = async (defaults: Defaults, boardName: string) => {
	const boardDefaults = defaults.defaults[boardName];
	if (!defaults.defaults[boardName])
		throw `invalid board selected - ${boardName}`;

	const args = new Map<
		string,
		{
			value: string | number | boolean;
			type: "pin" | "number" | "raw" | "string";
		}
	>();

	const add = (
		key: string,
		value: string | boolean | number | undefined,
		type: "pin" | "number" | "raw" | "string"
	) => {
		if (value !== undefined) args.set(key, { value, type });
	};

	// add("LED_PIN", boardDefaults.values.LED_PIN, "pin");
	add("BOARD", boardDefaults.values.BOARD, "raw");
	// add("LED_INVERTED", boardDefaults.values.LED_INVERTED, "raw");

	if (boardDefaults.values.SENSORS) {
		boardDefaults.values.SENSORS.forEach((sensor, index) => {
			if (index === 0) {
				add("IMU", sensor.imu, "raw");
				add("PIN_IMU_INT", sensor.int, "pin");
				add("IMU_ROTATION", sensor.rotation, "raw");
				if (sensor.protocol === "I2C") {
					add("PRIMARY_IMU_ADDRESS_ONE", sensor.address, "number");
				}
			}
			if (index === 1) {
				add("SECOND_IMU", sensor.imu, "raw");
				add("PIN_IMU_INT_2", sensor.int, "pin");
				add("SECOND_IMU_ROTATION", sensor.rotation, "raw");
				if (sensor.protocol === "I2C") {
					add("SECONDARY_IMU_ADDRESS_TWO", sensor.address, "number");
				}
			}

			if (sensor.protocol === "I2C") {
				add("PIN_IMU_SDA", sensor.sda, "pin");
				add("PIN_IMU_SCL", sensor.scl, "pin");
			}
		});
	}

	const battery = boardDefaults.values.BATTERY;
	if (battery) {
		add("BATTERY_MONITOR", battery.type, "raw");
		if (battery.type === "BAT_EXTERNAL") {
			// add("PIN_BATTERY_LEVEL", battery.pin, "pin");
			add("BATTERY_SHIELD_RESISTANCE", battery.shieldR, "number");
			add("BATTERY_SHIELD_R1", battery.r1, "number");
			add("BATTERY_SHIELD_R2", battery.r2, "number");
		}
	}

	console.log(
		Array.from(args.entries())
			.map(([key, { value, type }]) => {
				if (type === "pin")
					return `-D${key}=${
						value.toString().match(/[AD]/) ? `'${value}'` : value
					}`;
				if (type === "string") return `-D${key}='${value}'`;
				if (type === "raw" || type === "number")
					return `-D${key}=${value}`;
			})
			.join(" ")
	);
};

program
	.name("build boards")
	.option(
		"-s, --schema <schema>",
		"path to schema",
		"./board-defaults.schema.json"
	)
	.option("-b, --board <board>", "path to schema")
	.option(
		"-d, --defaults <defaults>",
		"path to defaults",
		"./board-defaults.json"
	)
	.action(async (args) => {
		if (!args.schema || !args.defaults)
			throw "schema and defaults path no set";

		const schema = JSON.parse(
			await readFile(
				resolve(join(__dirname, "../", args.schema)),
				"utf-8"
			)
		);
		const defaults = JSON.parse(
			await readFile(
				resolve(join(__dirname, "../", args.defaults)),
				"utf-8"
			)
		) as Defaults;
		const validate = ajv.validate(schema, defaults);
		if (!validate) throw "invalid defaults format against schema";

		if (args.board) {
			buildBoard(defaults, args.board);
		} else {
			for (const boardName of Object.keys(defaults.defaults)) {
				buildBoard(defaults, boardName);
			}
		}
	});

program.parse();

process.on("unhandledRejection", (err) => {
	console.error("Error", err);
});
