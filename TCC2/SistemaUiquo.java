Se cálculoDoPID está habilitado Então

	Se velocidadeAtual está numa faixa constante Então
		IncrementaTempoDeInterações
	SeNão
		DefineFaixaDeVelocidade(velocidadeAtual)
		ZeraTempoDeInterações
	FimSe

	Se chegou ao tempo de interações Então
		LigaLED
		DefineTargetVelocidade(velocidadeAtual)
		DefineServoIgualPosicaoDoCabo
	FimSe

	Se led estiver ligado Então
		EsperaParaHabilitarUsoDoPID
		Se chegou ao tempo de resposta Então
			Se motorista soltou o pedal Então
				HabilitaCálculoDoPID
			FimSe
		SeNão
			DesligaLED
			ZeraTemposDeInterações
		FimSe
	FimSe

SeNão

	Se aceleração ultrapassar target Então
		DesabilitaCálculoDoPID
		DesabilitaUsoDoPID
		DesligaLED
		ZeraTemposDeInterações
		DefineFaixaDeVelocidade(velocidadeAtual)
	FimSe

FimSe