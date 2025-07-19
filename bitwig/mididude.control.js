loadAPI(18);

host.setShouldFailOnDeprecatedUse(true);
host.defineController(
	"matelab",
	"mididude",
	"0.1",
	"acf17377-a010-4793-b0e8-45f4a0e0066c",
);
host.defineMidiPorts(1, 1);
host.addDeviceNameBasedDiscoveryPair(["Matelab"], ["Mididude"]);

let values = [0, 0, 0, 0, 0, 0, 0, 0];
let remoteControlCursor;
let outPort;
let sendUpdates = [];

function init() {
	let inPort = host.getMidiInPort(0);
	outPort = host.getMidiOutPort(0);
	inPort.setMidiCallback(onMidi0);

	let cursorTrack = host.createCursorTrack(0, 0);
	let cursorDevice = cursorTrack.createCursorDevice();
	remoteControlCursor = cursorDevice.createCursorRemoteControlsPage(8);

	for (let j = 0; j < remoteControlCursor.getParameterCount(); j++) {
		let valueFn = onValueChange.bind(this, j);
		let param = remoteControlCursor.getParameter(j);
		param.markInterested();
		param.setIndication(true);
		param.value().addValueObserver(16384, valueFn);
	}
	println("Mididude initialized!");
}

function onValueChange(paramIdx, value) {
	sendUpdates.push({
		paramIdx: paramIdx,
		value: value,
	});
}

function onMidi0(status, data1, data2) {
	// printMidi(status, data1, data2);
	let paramIdx;

	if (isChannelController(status)) {
		if (data1 >= 9 && data1 <= 16) {
			paramIdx = data1 - 9;
			values[paramIdx] = (values[paramIdx] & 0x7f) + (data2 << 7);
		}
		if (data1 >= 41 && data1 <= 49) {
			paramIdx = data1 - 41;
			values[paramIdx] = (values[paramIdx] & 0x3f80) + data2;
			remoteControlCursor.getParameter(paramIdx).value().set(values[paramIdx] / 16384);
		}
	}
	return true;
}

function flush() {
	for (let update of sendUpdates) {
		let hsb = update.value >> 7;
		let lsb = update.value & 127;
		outPort.sendMidi(0xb5, 9 + update.paramIdx, hsb);
		outPort.sendMidi(0xb5, 9 + update.paramIdx + 32, lsb);
	}
	sendUpdates = [];
}

function exit() { }
