#pragma once

/// OSC Message Protocol

// send
const std::string OSC_HELLO = "/hi";
const std::string OSC_INFO = "/info";
const std::string OSC_BYE = "/bye";
const std::string OSC_FITNESS = "/fit";

const std::string OSC_ARTIFACT_START = "/art/start/";
const std::string OSC_ARTIFACT_PART = "/art/part/";
const std::string OSC_ARTIFACT_END = "/art/end/";
const std::string OSC_ARTIFACT_INFO = "/art/info/";

// receive
const std::string OSC_HELLO_IN = "hi";
const std::string OSC_INFO_IN = "info";
const std::string OSC_BYE_IN = "bye";
const std::string OSC_ACTIVATION = "act";
const std::string OSC_ACTIVATION_BUF = "actbuf";
const std::string OSC_ID = "id";

const std::string OSC_START = "start";
const std::string OSC_PART = "part";
const std::string OSC_END = "end";
