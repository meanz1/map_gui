const fs = require('fs').promises;
const path = require('path');
const express = require('express');
const asyncify = require('express-asyncify');
const bodyParser = require('body-parser');
const app = asyncify(express());
app.use(bodyParser.urlencoded({
    extended: false
}));
app.use(express.static('.'))

const ip = require("ip");

const hostIp = ip.address();

const port = 8080;

const exec = require("child_process").exec;
const execSync = require("child_process").execSync;

app.listen(port, hostIp, () => {
    console.log(`Server Running at ${hostIp}:${port}`)
})
