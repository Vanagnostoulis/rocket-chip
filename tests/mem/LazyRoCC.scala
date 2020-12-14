class ExampleAccel (opcodes: OpcodeSet)(implicit p: Parameters) extends LazyRoCC(opcodes) {
  override lazy val module = new ExampleAccelModuleImp(this)
}

@chiselName
class ExampleAccelModuleImp(outer: ExampleAccel)(implicit p: Parameters) extends LazyRoCCModuleImp(outer)
with HasCoreParameters {
  val rs2_reg     = Reg(chiselTypeOf(io.cmd.bits.rs2))
  val rd_reg      = Reg(chiselTypeOf(io.cmd.bits.inst.rd))
  val funct_reg   = Reg(chiselTypeOf(io.cmd.bits.inst.funct))
  val addr_reg    = Reg(chiselTypeOf(io.mem.req.bits.addr))
  val mem_data    = Reg(chiselTypeOf(io.mem.resp.bits.data))

  val s_idle :: s_mem_req :: s_mem_resp :: s_resp :: Nil = Enum(4) 
  val state = RegInit(s_idle)

  // eimai ready na dextw entoli mono stin idle
  io.cmd.ready := (state === s_idle)

  // otan erthei entoli pare auta pou xreiazomai 
  when (io.cmd.fire()){
    addr_reg  := io.cmd.bits.rs1
    rs2_reg   := io.cmd.bits.rs2
    rd_reg    := io.cmd.bits.inst.rd
    funct_reg := io.cmd.bits.inst.funct
    state     := s_mem_req

  }
  // when a send a request to memory wait for response 
  when (io.mem.req.fire()){ 
    printf(p"send request with add $addr_reg rs2 $rs2_reg func $funct_reg cmd ${io.mem.req.bits.cmd}\n")
    state := s_mem_resp 
  }
  // when for a load request 
  when (state === s_mem_resp && io.mem.resp.valid && io.mem.resp.bits.has_data){ 
    printf(p"return from load request with add $addr_reg rs2 $rs2_reg func $funct_reg\n")
    mem_data := io.mem.resp.bits.data
    state    := s_resp 
  }.elsewhen (state === s_mem_resp && io.mem.resp.valid && !io.mem.resp.bits.has_data){ 
    printf(p"return from store request with add $addr_reg rs2 $rs2_reg func $funct_reg\n")
    state    := s_idle
  }
  
  when (io.resp.fire()) { 
    printf(p"response to core with load req add $addr_reg rs2 $rs2_reg func $funct_reg\n")
    state := s_idle 
  }

  io.mem.req.valid      := (state === s_mem_req)
  io.mem.req.bits.addr  := addr_reg
  io.mem.req.bits.tag   := addr_reg(4,0) //maybe find another way for tagging
  io.mem.req.bits.cmd   := Mux(funct_reg === 1.U, M_XRD, M_XWR)
  io.mem.req.bits.size  := log2Ceil(8).U
  io.mem.req.bits.signed:= false.B 
  io.mem.req.bits.phys  := false.B
  io.mem.req.bits.data  := Mux(funct_reg === 1.U, 0.U, rs2_reg)


  io.resp.valid       := (state === s_resp)
  io.resp.bits.rd     := rd_reg
  io.resp.bits.data   := mem_data

  io.interrupt := false.B
 }

