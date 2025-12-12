class MotorController {
  constructor() {
    this.port = null;
    this.reader = null;
    this.writer = null;
    this.encoder = new TextEncoder();
    this.decoder = new TextDecoder();
    this.buffer = "";
    this.onData = null;
    this.onRawData = null;
    this.isConnected = false;
    this._internalListeners = [];

    // --- Queue System ---
    this.commandQueue = [];
    this.isProcessing = false;
    this.currentCmdPromise = null; // ตัวเก็บ Promise ของคำสั่งที่กำลังรัน
    this.currentExpectations = null; // เก็บว่าคำสั่งปัจจุบันรอ Code อะไร
    this.onQueueUpdate = null;
    this._internalListeners = [];

    // ✅ เพิ่มตัวนี้: เก็บสถานะล่าสุดของมอเตอร์แต่ละตัว
    this.motorStates = {
      Motor1: { status: "UNKNOWN" },
      Motor2: { status: "UNKNOWN" },
      Motor3: { status: "UNKNOWN" },
    };

    // ===== Tool Controller System =====
    this.toolName = null; // ชื่อ Tool ปัจจุบัน
    this.toolCommands = []; // รายการคำสั่งของ Tool
    this.toolRefreshInterval = null; // Interval สำหรับ refresh
    this.onToolUpdate = null; // Callback เมื่อ Tool อัพเดท
    this._pendingToolQuery = null; // Promise resolver สำหรับรอ response
  }

  async connect() {
    if (!navigator.serial) {
      alert("Web Serial API not supported.");
      return false;
    }
    try {
      this.port = await navigator.serial.requestPort();
      await this.port.open({ baudRate: 115200 });
      this.isConnected = true;
      this.readLoop();
      return true;
    } catch (error) {
      console.error("Connection failed:", error);
      return false;
    }
  }

  async disconnect() {
    if (this.reader) {
      await this.reader.cancel();
      this.reader = null;
    }
    if (this.writer) {
      this.writer.releaseLock();
      this.writer = null;
    }
    if (this.port) {
      await this.port.close();
      this.port = null;
    }
    this.isConnected = false;
  }

  async readLoop() {
    while (this.port.readable && this.isConnected) {
      this.reader = this.port.readable.getReader();
      try {
        while (true) {
          const { value, done } = await this.reader.read();
          if (done) break;
          const text = this.decoder.decode(value);
          this.handleData(text);
        }
      } catch (error) {
        console.error("Read error:", error);
      } finally {
        this.reader.releaseLock();
      }
    }
  }
  // เพิ่มฟังก์ชันนี้ลงใน Class MotorController
  async waitForIdle() {
    console.log("Waiting for motors to be IDLE...");
    while (true) {
      // 1. ส่งคำสั่ง 'd' เพื่อขอสถานะล่าสุด (Queue จะรอให้ตอบกลับมาครบ 3 ตัว)
      await this.send("d");

      // 2. เช็คจากข้อมูลล่าสุดที่เราเพิ่งได้รับมา
      const m1Ok = this.motorStates.Motor1?.status === "IDLE";
      const m2Ok = this.motorStates.Motor2?.status === "IDLE";
      const m3Ok = this.motorStates.Motor3?.status === "IDLE";

      // 3. ถ้าหยุดนิ่งครบทุกตัวแล้ว -> ไปต่อได้!
      if (m1Ok && m2Ok && m3Ok) {
        console.log("All motors are IDLE.");
        break;
      }

      // 4. ถ้ายังไม่หยุด (กำลังเด้งอยู่) -> รอ 0.5 วิ แล้วถามใหม่
      await new Promise((r) => setTimeout(r, 500));
    }
  }

  handleData(text) {
    this.buffer += text;
    const lines = this.buffer.split("\n");
    this.buffer = lines.pop();

    for (const line of lines) {
      const trimmedLine = line.trim();
      if (trimmedLine === "") continue;

      if (this.onRawData) this.onRawData(trimmedLine);

      if (trimmedLine.startsWith("{")) {
        try {
          const data = JSON.parse(trimmedLine);
          
          // 🔍 Debug: Log all parsed JSON
          console.log("[RX]", data);

          // ✅ เพิ่มตรงนี้: อัปเดตสถานะล่าสุดเก็บไว้
          if (data.motor || data.motorName) {
            const name = data.motor || data.motorName;
            // เช็คเผื่อชื่อมาแปลกๆ หรือ map ให้ตรงกับ key
            if (this.motorStates[name]) {
              this.motorStates[name] = { ...this.motorStates[name], ...data };
            }
          }
          if (this.onData) this.onData(data);

          // ===== Handle AUX Tool Response =====
          // Case 1: Wrapped response { type: "AUX", message: "..." }
          if (data.type === "AUX") {
            this._handleAuxResponse(data);
          }
          
          // Case 2: Direct tool query response (m?) - has "commands" array
          // Tool ส่ง {"type":"INFO","code":101,"name":"...","commands":[...]}
          if (data.commands && Array.isArray(data.commands)) {
            console.log("[Direct Tool Query Response]", data);
            this._handleAuxResponse(data); // Use same handler
          }
          
          // Case 3: Direct response from tool command (not wrapped)
          // Tool ส่งตรงๆ เช่น {"type":"SUCCESS","code":201,...}
          // ตรวจสอบว่าเป็น response จาก tool โดยดู code range (100-402 คือ Torom codes)
          // และต้องไม่ใช่ motor response (motor response มี motor field)
          if (data.code && data.code >= 100 && data.code <= 402 && 
              data.type !== "AUX" && !data.motor && !data.motorName) {
            // ถ้ากำลังรอ AUX response อยู่ ให้ถือว่าเป็น tool response
            if (this.currentExpectations?.type === "AUX") {
              console.log("[AUX Direct Command Response]", data);
              this._checkQueueExpectationsForAux(data);
            }
          }

          // --- Logic การเช็ค Response เพื่อปลดล็อค Queue ---
          this._checkQueueExpectations(data);

          // Handle internal listeners (for manual waits if needed)
          if (this._internalListeners.length > 0) {
            this._internalListeners = this._internalListeners.filter(
              (l) => !l(data)
            );
          }
        } catch (e) {
          console.warn("Parse error", e, trimmedLine);
        }
      }
    }
  }
  
  /**
   * Handle direct AUX response (not wrapped)
   */
  _checkQueueExpectationsForAux(data) {
    if (!this.currentExpectations || !this.currentCmdPromise) return;
    if (this.currentExpectations.type !== "AUX") return;
    
    const code = data.code;
    
    // Error codes
    if (code === 400 || code === 401 || code === 402) {
      console.log("[AUX Error]", data);
      this.currentCmdPromise.reject(
        new Error(`Tool Error: ${data.message} (Code ${code})`)
      );
      return;
    }
    
    // Success codes: 200 (SUCCESS), 201 (TARGET_REACHED)
    if (code === 200 || code === 201) {
      console.log("[AUX Success]", data);
      this.currentExpectations.count--;
      if (this.currentExpectations.count <= 0) {
        this.currentCmdPromise.resolve(data);
      }
    }
  }

  /**
   * ฟังก์ชันหัวใจหลัก: ตรวจสอบว่า Data ที่เข้ามา ตรงกับที่คำสั่งปัจจุบันรออยู่ไหม
   */
  _checkQueueExpectations(data) {
    if (!this.currentExpectations || !this.currentCmdPromise) return;

    const exp = this.currentExpectations;
    let code = data.code;

    // === Handle AUX Response ===
    // AUX response มาในรูป { type: "AUX", message: "{...json from tool...}" }
    // ต้องดึง code จาก message ที่ parse แล้ว
    if (exp.type === "AUX" && data.type === "AUX") {
      try {
        let auxData = typeof data.message === "string" 
          ? JSON.parse(data.message) 
          : data.message;
        code = auxData.code;
      } catch (e) {
        console.warn("Failed to parse AUX message for queue check");
        return;
      }
    }

    // 1. Error Codes (Fatal) - อันนี้คงเดิม
    if (code === 406 || code === 407 || code === 403 || code === 400 || code === 401 || code === 402) {
      this.currentCmdPromise.reject(
        new Error(`Device Error: ${data.message} (Code ${code})`)
      );
      return;
    }

    // 2. ✨ Logic ใหม่: รวมพลัง Success และ Stop
    // ถ้าเป็นคำสั่ง MOVE ให้ถือว่า 211(ถึง), 212(สั่งหยุด), 213(ชนลิมิต) คือ "จบงานของมอเตอร์ตัวนั้น"
    const isMoveFinish =
      exp.type === "MOVE" && (code === 211 || code === 212 || code === 213);
    
    // AUX Tool: 200 (SUCCESS), 201 (TARGET_REACHED) คือเสร็จ
    const isAuxFinish =
      exp.type === "AUX" && (code === 200 || code === 201);

    // ถ้าเป็นคำสั่งอื่น ดูตามโพยที่จดมา
    const isExpected = exp.codes.includes(code);

    if (isMoveFinish || isAuxFinish || isExpected) {
      // ลดจำนวนที่ต้องรอลง 1 แต้ม
      exp.count--;

      // ถ้าครบทุกตัวแล้ว (เหลือ 0) -> ปลดล็อค!
      if (exp.count <= 0) {
        this.currentCmdPromise.resolve(data);
      }
    }
  }

  async sendImmediate(command) {
    if (!this.port || !this.port.writable) return;

    try {
      // 1. แย่งกุญแจ (Writer) มาเลย เพราะ Web Serial ยอมให้เขียนได้
      // แม้ว่าจะมีคำสั่งอื่นรอ Response อยู่ (ตราบใดที่คำสั่งนั้นเขียนเสร็จไปแล้ว)
      const writer = this.port.writable.getWriter();
      const cmdToSend = command.endsWith("\n") ? command : command + "\n";
      await writer.write(this.encoder.encode(cmdToSend));
      writer.releaseLock();

      // 2. ถ้าเป็นคำสั่ง STOP (s) หรือ EMERGENCY (e) ต้องล้างคิวทิ้งด้วย!
      // เพราะถ้าเราสั่งหยุดแล้ว คำสั่ง Move ที่รอคิวอยู่ก็ไม่ควรทำต่อแล้ว
      if (
        command === "s" ||
        command.startsWith("s") ||
        command === "e" ||
        command.startsWith("e") ||
        command.includes(":s") ||
        command.includes(":e")
      ) {
        console.log("Stop command detected! Clearing queue.");

        // ล้าง Array คิว
        this.commandQueue = [];

        // แจ้ง UI ให้ล้างรายการ
        if (this.onQueueUpdate) this.onQueueUpdate("clear", null);
      }
    } catch (error) {
      console.error("Send Immediate Error:", error);
    }
  }

  // ✅ ฟังก์ชัน send ปกติ (สำหรับคำสั่งทั่วไปที่ต้องเข้าคิว)
  async send(command) {
    // ... (โค้ดเดิมเป๊ะๆ สำหรับคำสั่ง Move/Config) ...
    return new Promise((resolve, reject) => {
      const id = Date.now() + Math.random();
      const expectations = this._parseCommandExpectations(command);
      const cmdItem = {
        id,
        command,
        resolve,
        reject,
        expectations,
        timestamp: new Date(),
      };
      this.commandQueue.push(cmdItem);
      if (this.onQueueUpdate) this.onQueueUpdate("add", cmdItem);
      this._processQueue();
    });
  }

  // ... (ฟังก์ชัน _processQueue, _finishCommand, _parseCommandExpectations เหมือนเดิม) ...

  // ================= High Level API (แก้ตรงนี้!) =================

  // กลุ่ม 1: คำสั่งทั่วไป (เข้าคิว)
  async moveAbsolute(motorId, position) {
    await this.send(`${motorId}:${position}`);
  }
  async moveRelative(motorId, steps) {
    const sign = steps >= 0 ? "+" : "";
    await this.send(`${motorId}:${sign}${steps}`);
  }
  async moveAll(x, y, z = 0) {
    await this.send(`${x},${y},${z}`);
  }
  async setHome(motorId = 0) {
    if (motorId === 0) await this.send("h");
    else await this.send(`${motorId}:h`);
  }
  async enable(motorId = 0) {
    if (motorId === 0) await this.send("on");
    else await this.send(`${motorId}:on`);
  }
  async disable(motorId = 0) {
    if (motorId === 0) await this.send("off");
    else await this.send(`${motorId}:off`);
  }
  async setSpeed(speed) {
    await this.send(`x${speed}`);
  }
  async setAcceleration(accel) {
    await this.send(`a${accel}`);
  }
  async setLimitCompensation(ratio) {
    await this.send(`i${ratio}`);
  }

  // กลุ่ม 2: คำสั่ง VVIP (แซงคิวทันที!) 🚀
  async stop(motorId = 0) {
    if (motorId === 0) await this.sendImmediate("s");
    else await this.sendImmediate(`${motorId}:s`);
  }

  async emergencyStop(motorId = 0) {
    if (motorId === 0) await this.sendImmediate("e");
    else await this.sendImmediate(`${motorId}:e`);
  }

  // Status (d) อยากให้อัปเดตทันที ไม่ต้องรอคิว Move เสร็จ
  async getDetailedStatus(motorId = 0) {
    if (motorId === 0) await this.sendImmediate("d");
    else await this.sendImmediate(`${motorId}:d`);
  }

  async _processQueue() {
    if (this.isProcessing || this.commandQueue.length === 0) return;

    this.isProcessing = true;
    const currentItem = this.commandQueue[0];

    try {
      if (!this.port || !this.port.writable)
        throw new Error("Port not writable");

      if (this.onQueueUpdate) this.onQueueUpdate("process", currentItem);

      // Setup การรอ Response
      this.currentCmdPromise = {
        resolve: (res) => {
          this._finishCommand(currentItem, "complete", res);
        },
        reject: (err) => {
          this._finishCommand(currentItem, "error", err);
        },
      };
      this.currentExpectations = currentItem.expectations;

      // ส่งข้อมูลออกไป
      const writer = this.port.writable.getWriter();
      const cmdToSend = currentItem.command.endsWith("\n")
        ? currentItem.command
        : currentItem.command + "\n";
      await writer.write(this.encoder.encode(cmdToSend));
      writer.releaseLock();

      // *** จุดต่าง: ไม่ resolve ทันที แต่รอให้ _checkQueueExpectations เรียก resolve ***
    } catch (error) {
      this._finishCommand(currentItem, "error", error);
    }
  }

  _finishCommand(item, status, result) {
    // ล้างสถานะ
    this.currentCmdPromise = null;
    this.currentExpectations = null;

    // แจ้งผลกลับไปที่คนเรียก (await send(...))
    if (status === "error") item.reject(result);
    else item.resolve(result);

    // UI Update
    if (this.onQueueUpdate) this.onQueueUpdate(status, item);

    // เอาออกจากคิวและทำตัวต่อไป
    this.commandQueue.shift();
    this.isProcessing = false;

    // ทำต่อทันที
    if (this.commandQueue.length > 0) {
      this._processQueue();
    }
  }

  /**
   * วิเคราะห์คำสั่งว่าต้องรอ Code อะไร และกี่ครั้ง
   */
  _parseCommandExpectations(cmd) {
    cmd = cmd.trim();

    // --- 1. Control Commands ---
    if (cmd === "s" || cmd.endsWith(":s"))
      return { type: "STOP", codes: [212], count: 1 }; // Wait for Stop
    if (cmd === "e" || cmd.endsWith(":e"))
      return { type: "ESTOP", codes: [213], count: 1 }; // Wait for E-Stop
    if (cmd === "h" || cmd.endsWith(":h"))
      return { type: "HOME", codes: [206], count: this._countTargets(cmd) }; // Wait for Home Set
    if (cmd === "on" || cmd.endsWith(":on"))
      return { type: "ENABLE", codes: [207], count: this._countTargets(cmd) };
    if (cmd === "off" || cmd.endsWith(":off"))
      return { type: "DISABLE", codes: [208], count: this._countTargets(cmd) };

    // --- 2. Config Commands ---
    if (cmd.startsWith("x") || cmd.includes(":x"))
      return { type: "SPEED", codes: [205], count: this._countTargets(cmd) };
    if (cmd.startsWith("a") || cmd.includes(":a"))
      return { type: "ACCEL", codes: [209], count: this._countTargets(cmd) };
    if (cmd.startsWith("i")) return { type: "CONFIG", codes: [300], count: 1 };
    
    // --- AUX Tool Commands ---
    // Torom Tool returns: 200 (SUCCESS), 201 (TARGET_REACHED), 100 (INFO)
    // isQueue commands wait for 200/201, non-queue don't wait
    if (cmd.startsWith("m")) return { type: "AUX", codes: [200, 201], count: 1 };

    // --- 3. Info Commands ---
    if (cmd.includes("d"))
      return {
        type: "STATUS",
        codes: [200, 201, 202, 203],
        count: this._countTargets(cmd),
      };
    if (cmd.includes("p")) return { type: "POS", codes: [210], count: 1 }; // p returns 1 JSON even for multiple motors
    if (cmd.includes("l"))
      return {
        type: "LIMIT",
        codes: [404, 405],
        count: this._countTargets(cmd),
      };

    // --- 4. Movement (Default) ---
    // ถ้าไม่เข้าเคสบนๆ น่าจะเป็นตัวเลข (Move) เช่น "1000" หรือ "1:100" หรือ "+100"
    return { type: "MOVE", codes: [211], count: this._countTargets(cmd) };
  }

  _countTargets(cmd) {
    // กรณีระบุเป้าหมายชัดเจน เช่น "1:..." -> 1 ตัว
    if (cmd.match(/^[1-3]:/)) return 1;

    // กรณีส่งรวม เช่น "100,200,300" -> นับตามจำนวนเครื่องหมายลูกน้ำ + 1
    if (cmd.includes(",")) {
      return cmd.split(",").length;
    }

    // กรณีไม่มี Prefix และไม่มีลูกน้ำ (เช่น "s", "h", "on") Firmware ถือว่า All Motors
    // แต่ต้องระวังพวก Config บางตัว Firmware อาจตอบกลับมาแค่ 1 ครั้ง หรือ 3 ครั้ง ขึ้นอยู่กับ Implementation
    // จาก Firmware main2.cpp:
    // - stop/estop/home/on/off (All) -> Loop เรียกทีละตัว -> ส่ง JSON 3 รอบ -> Count 3
    // - setSpeed/Accel (All) -> Loop -> Count 3

    // ดังนั้นถ้าเป็นคำสั่ง Global (ไม่มี :) ให้เหมาเป็น 3 ไว้ก่อน (ถ้าบอร์ดเรามี 3 แกน)
    // **ปรับปรุงตาม main2.cpp**: ถ้าส่ง 's' เฉยๆ -> motorX.stop(); motorY.stop(); motorZ.stop(); -> แต่ละอัน return 212 -> รวมเป็น 3
    return 3;
  }

  // ================= Tool Controller Methods =================

  /**
   * Handle AUX response from device (wrapped or direct)
   */
  _handleAuxResponse(data) {
    console.log("[_handleAuxResponse]", data);
    
    try {
      let auxData;

      // Case 1: Wrapped response { type: "AUX", message: "..." or {...} }
      if (data.type === "AUX" && data.message !== undefined) {
        if (typeof data.message === "string") {
          auxData = JSON.parse(data.message);
        } else {
          auxData = data.message;
        }
      } 
      // Case 2: Direct response (data is already the tool response)
      else {
        auxData = data;
      }
      
      console.log("[AUX Parsed]", auxData);

      // Handle pending tool query (m?)
      if (this._pendingToolQuery) {
        // ตรวจสอบว่าเป็น INFO response (code 101 สำหรับ m?)
        if (auxData.type === "INFO" && auxData.commands) {
          console.log("[Tool Query Response]", auxData);
          this._pendingToolQuery.resolve(auxData);
          this._pendingToolQuery = null;
          return;
        }
        // หรือ code 101
        if (auxData.code === 101 && auxData.commands) {
          console.log("[Tool Query Response by Code]", auxData);
          this._pendingToolQuery.resolve(auxData);
          this._pendingToolQuery = null;
          return;
        }
      }
    } catch (e) {
      console.error("Error parsing AUX response:", e, data);
    }
  }
  
  /**
   * Handle direct tool response (not wrapped in AUX)
   */
  _handleDirectToolResponse(data) {
    console.log("[Direct Tool Response]", data);
    
    // Handle pending tool query
    if (this._pendingToolQuery && data.commands) {
      this._pendingToolQuery.resolve(data);
      this._pendingToolQuery = null;
    }
  }

  /**
   * Query available tool commands from AUX device
   * @returns {Promise<{name: string, commands: Array}>}
   */
  async queryToolCommands() {
    if (!this.isConnected) {
      throw new Error("Not connected");
    }

    // สร้าง Promise เพื่อรอ response จาก AUX
    const responsePromise = new Promise((resolve, reject) => {
      this._pendingToolQuery = { resolve, reject };

      // Timeout หลัง 5 วินาที
      setTimeout(() => {
        if (this._pendingToolQuery) {
          this._pendingToolQuery = null;
          reject(new Error("Timeout waiting for AUX response"));
        }
      }, 5000);
    });

    // ส่งคำสั่ง m?
    await this.sendImmediate("m?");

    // รอ response
    const auxData = await responsePromise;

    // ตรวจสอบว่ามี commands array
    if (auxData.commands && Array.isArray(auxData.commands)) {
      return {
        name: auxData.name || "AUX Tool",
        commands: auxData.commands,
      };
    } else {
      throw new Error("No commands found in AUX response");
    }
  }

  /**
   * Start auto-refresh tool commands every 5 seconds
   */
  startToolRefresh() {
    if (this.toolRefreshInterval) return; // Already running

    console.log("Tool refresh started (5s interval)");

    // Query immediately first time
    this._refreshTools();

    // Then refresh every 5 seconds
    this.toolRefreshInterval = setInterval(() => {
      this._refreshTools();
    }, 5000);
  }

  /**
   * Stop auto-refresh tool commands
   */
  stopToolRefresh() {
    if (this.toolRefreshInterval) {
      clearInterval(this.toolRefreshInterval);
      this.toolRefreshInterval = null;
      console.log("Tool refresh stopped");
    }
  }

  /**
   * Internal: Refresh tool commands
   */
  async _refreshTools() {
    if (!this.isConnected) return;

    try {
      const result = await this.queryToolCommands();

      // ตรวจสอบว่าชื่อเปลี่ยนไหม ถ้าเหมือนเดิมไม่ต้องอัพเดท UI
      if (this.toolName === result.name) {
        // ชื่อเหมือนเดิม - ตรวจสอบว่า commands เปลี่ยนไหม
        const isSame =
          JSON.stringify(this.toolCommands) === JSON.stringify(result.commands);
        if (isSame) {
          console.log("Tool unchanged, skipping UI update");
          return;
        }
      }

      // อัพเดทข้อมูล
      this.toolName = result.name;
      this.toolCommands = result.commands;

      console.log(
        `Tool updated: ${result.name} (${result.commands.length} commands)`
      );

      // แจ้ง UI
      if (this.onToolUpdate) {
        this.onToolUpdate(this.toolName, this.toolCommands);
      }
    } catch (error) {
      console.error("Tool refresh error:", error.message);
      // Don't clear existing tools on error
    }
  }

  /**
   * Execute a tool command
   * @param {object} cmd - Command definition from toolCommands
   * @param {string|number} value - Optional value for commands with input
   * @returns {Promise}
   */
  async executeToolCommand(cmd, value = null) {
    if (!cmd || !cmd.cmd) {
      throw new Error("Invalid command");
    }

    let commandStr = cmd.cmd;

    // ถ้ามี input - ถ้ามี <val> ให้ replace, ไม่งั้นต่อท้าย
    if (cmd.inputType && cmd.inputType !== "none" && value !== null) {
      if (commandStr.includes("<val>")) {
        commandStr = commandStr.replace("<val>", value);
      } else {
        commandStr = `${cmd.cmd}${value}`;
      }
    }

    // ส่งคำสั่ง m<command> ไปยัง AUX
    const fullCommand = `m${commandStr}`;

    console.log(`Tool execute: ${fullCommand} (isQueue: ${cmd.isQueue})`);

    if (cmd.isQueue) {
      // ใช้ queue system
      return await this.send(fullCommand);
    } else {
      // ส่งตรง ไม่ผ่าน queue
      await this.sendImmediate(fullCommand);
      return null; // sendImmediate doesn't return response
    }
  }

  /**
   * Get current tool info
   */
  getToolInfo() {
    return {
      name: this.toolName,
      commands: this.toolCommands,
      isAvailable: this.toolCommands.length > 0,
    };
  }

  // Calibration แบบใหม่: ง่ายขึ้นมาก เพราะคำสั่ง send มันรอ response ให้เองแล้ว
  // Calibration แบบ Full Range: หา 0 และหา Max
  async calibration() {
    if (!this.isConnected) throw new Error("Not connected");

    console.log("Calibration: Started (Full Range)");

    // 1. เตรียมความพร้อม (ช้าๆ หน่อย จะได้ไม่กระแทกแรง)
    await this.setSpeed(1);
    await this.setAcceleration(0.5);

    // =========================================
    // Phase 1: หาจุด Home (Min / 0)
    // =========================================
    console.log("Phase 1: Finding Home...");
    // สั่งวิ่งไปทางลบเยอะๆ (เช่น -50000) เพื่อให้ชนสวิตช์ซ้ายแน่นอน
    await this.moveAll(-50000, -50000, -50000);

    // รอจนกว่าจะชนและเด้งกลับจนนิ่ง (Firmware Auto-bounce)
    await this.waitForIdle();

    // ตั้งค่า 0 ตรงนี้
    await this.setHome();
    console.log("Home Set (0,0,0)");

    // =========================================
    // Phase 2: หาจุด Max (Measure Length)
    // =========================================
    console.log("Phase 2: Measuring Axis Length...");
    // สั่งวิ่งไปทางบวกเยอะๆ (เช่น +50000) เพื่อให้ชนสวิตช์ขวาแน่นอน
    // นายท่านอาจจะเปลี่ยนเลข 50000 เป็นเลขที่มากกว่าความยาวเครื่องจริง
    await this.moveAll(50000, 50000, 50000);

    // รอจนกว่าจะชนและเด้งกลับจนนิ่ง
    await this.waitForIdle();

    // =========================================
    // Phase 3: อ่านค่าและจบงาน
    // =========================================
    // ตอนนี้ตำแหน่งปัจจุบัน คือค่า Max ของเครื่อง (ที่ลบระยะ Bounce แล้ว)
    await this.getDetailedStatus();

    // ดึงค่าล่าสุดจาก State ที่เราเก็บไว้
    const m1Max = this.motorStates.Motor1?.position || 0;
    const m2Max = this.motorStates.Motor2?.position || 0;
    const m3Max = this.motorStates.Motor3?.position || 0;

    console.log(
      `Calibration Done! Machine Size: X=${m1Max}, Y=${m2Max}, Z=${m3Max}`
    );

    // (Optional) ถอยกลับมาตรงกลาง หรือจุด Safe Zone ที่ต้องการ (เช่น ถอยมา 10mm)
    // คำนวณหาจุดกึ่งกลาง (หาร 2)
    const mid1 = Math.floor(m1Max / 2);
    const mid2 = Math.floor(m2Max / 2);
    const mid3 = Math.floor(m3Max / 2);

    console.log(`Moving to Center: ${mid1}, ${mid2}, ${mid3}`);

    // 🚀 สั่งวิ่งไปตรงกลางพร้อมกันทีเดียว 3 แกน!
    await this.moveAll(mid1, mid2, mid3);
    await this.waitForIdle();

    // คืนค่าความเร็วปกติ
    await this.setSpeed(3);
    await this.setAcceleration(1);

    // ส่งค่า Max กลับไปให้ UI (เพื่อเอาไปวาด 3D Box)
    return {
      motor1: m1Max,
      motor2: m2Max,
      motor3: m3Max,
    };
  }
}
