Version 0.8.13 (2022-02-18)
========================
 * Improved driver monitoring
   * Retuned driver pose learner for relaxed driving positions
   * Added reliance on driving model to be more scene adaptive
   * Matched strictness between comma two and comma three
 * Improved performance in turns by compensating for the road bank angle
 * Improved camera focus on the comma two
 * AGNOS 4
   * ADB support
   * improved cell auto configuration
 * NEOS 19
   * package updates
   * stability improvements
 * Subaru ECU firmware fingerprinting thanks to martinl!
 * Hyundai Santa Fe Plug-in Hybrid 2022 support thanks to sunnyhaibin!
 * Mazda CX-5 2022 support thanks to Jafaral!
 * Subaru Impreza 2020 support thanks to martinl!
 * Toyota Avalon 2022 support thanks to sshane!
 * Toyota Prius v 2017 support thanks to CT921!
 * Volkswagen Caravelle 2020 support thanks to jyoung8607!

Version 0.8.12 (2021-12-15)
========================
 * 콤마 핫픽스 적용 등.
 * New driving model
   * Improved behavior around exits
 * Improved follow distance
 * Better longitudinal policy, especially in low speed traffic
 * New alert sounds
 * AGNOS 3
   * Display burn in mitigation
   * Improved audio amplifier configuration
   * System reliability improvements
   * Update Python to 3.8.10
 * Raw logs upload moved to connect.comma.ai
 * Fixed HUD alerts on newer Honda Bosch thanks to csouers!
 * Audi Q3 2020-21 support thanks to jyoung8607!
 * Lexus RC 2020 support thanks to ErichMoraga!