"""HMI 的自簽 TLS 憑證（★ 2026-11-03）。

為什麼要 https：瀏覽器只在「安全環境」（https 或 localhost）開放麥克風（getUserMedia）。
平板用 http://<機器人IP>:8080 連進來時，平板錄音功能（語音輸入＝browser）根本拿不到麥克風。

憑證放在 ~/.smartnav/hmi_tls/（不進 git；私鑰權限 600）。涵蓋：
  localhost、127.0.0.1、本機名稱、<本機名稱>.local、目前偵測到的所有區網 IPv4。
IP 換了（例如換到另一個 WiFi）就自動重簽，否則平板會看到「憑證與網址不符」。

自簽 = 平板第一次連線會出現安全警告，要按「繼續前往」（或把 cert.pem 安裝成信任的憑證）。
"""

import datetime
import ipaddress
import os
import socket
from pathlib import Path
from typing import Iterable, Optional, Tuple

CERT_NAME = "cert.pem"
KEY_NAME = "key.pem"
# iOS／macOS 對 TLS 伺服器憑證要求效期 <= 825 天、要標明 serverAuth，否則安裝成信任憑證也不認。
# 到期前 30 天內 HMI 啟動時會自動重簽（見 _cert_covers）。
VALID_DAYS = 825


def _wanted_names(lan_ips: Iterable[str]) -> Tuple[set, set]:
    host = socket.gethostname()
    dns = {"localhost", host, f"{host}.local"}
    ips = {"127.0.0.1", *lan_ips}
    return dns, ips


def _cert_covers(cert_path: Path, dns: set, ips: set) -> bool:
    from cryptography import x509

    try:
        cert = x509.load_pem_x509_certificate(cert_path.read_bytes())
        san = cert.extensions.get_extension_for_class(x509.SubjectAlternativeName).value
    except Exception:  # noqa: BLE001 —— 壞檔、沒有 SAN 都當成要重簽
        return False
    have_dns = set(san.get_values_for_type(x509.DNSName))
    have_ips = {str(ip) for ip in san.get_values_for_type(x509.IPAddress)}
    not_after = getattr(cert, "not_valid_after_utc", None) or cert.not_valid_after.replace(
        tzinfo=datetime.timezone.utc)
    fresh = not_after - datetime.datetime.now(datetime.timezone.utc) > datetime.timedelta(days=30)
    return dns <= have_dns and ips <= have_ips and fresh


def ensure_self_signed(tls_dir: Path, lan_ips: Iterable[str], logger=None) -> Optional[Tuple[str, str]]:
    """確保有一份涵蓋目前位址的自簽憑證，回傳 (cert 路徑, key 路徑)；做不出來回 None（只開 http）"""
    try:
        from cryptography import x509
        from cryptography.hazmat.primitives import hashes, serialization
        from cryptography.hazmat.primitives.asymmetric import ec
        from cryptography.x509.oid import NameOID
    except ImportError as exc:
        if logger:
            logger.error(f"沒有 python3-cryptography，無法產生 https 憑證（只開 http）：{exc}")
        return None

    tls_dir = Path(tls_dir)
    cert_path, key_path = tls_dir / CERT_NAME, tls_dir / KEY_NAME
    dns, ips = _wanted_names(lan_ips)
    if cert_path.exists() and key_path.exists() and _cert_covers(cert_path, dns, ips):
        return str(cert_path), str(key_path)

    tls_dir.mkdir(parents=True, exist_ok=True)
    key = ec.generate_private_key(ec.SECP256R1())
    name = x509.Name([x509.NameAttribute(NameOID.COMMON_NAME, f"SmartNav HMI ({socket.gethostname()})")])
    now = datetime.datetime.now(datetime.timezone.utc)
    alt = [x509.DNSName(d) for d in sorted(dns)]
    for ip in sorted(ips):
        try:
            alt.append(x509.IPAddress(ipaddress.ip_address(ip)))
        except ValueError:
            pass
    cert = (
        x509.CertificateBuilder()
        .subject_name(name)
        .issuer_name(name)
        .public_key(key.public_key())
        .serial_number(x509.random_serial_number())
        .not_valid_before(now - datetime.timedelta(minutes=5))
        .not_valid_after(now + datetime.timedelta(days=VALID_DAYS))
        .add_extension(x509.SubjectAlternativeName(alt), critical=False)
        .add_extension(x509.BasicConstraints(ca=True, path_length=None), critical=True)
        .add_extension(x509.KeyUsage(digital_signature=True, key_cert_sign=True, crl_sign=False,
                                     content_commitment=False, key_encipherment=False, data_encipherment=False,
                                     key_agreement=False, encipher_only=False, decipher_only=False), critical=True)
        .add_extension(x509.ExtendedKeyUsage([x509.oid.ExtendedKeyUsageOID.SERVER_AUTH]), critical=False)
        .sign(key, hashes.SHA256())
    )
    # 私鑰先以 600 建檔再寫入，避免出現「一瞬間別人讀得到」的空窗
    fd = os.open(key_path, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
    with os.fdopen(fd, "wb") as fp:
        fp.write(key.private_bytes(serialization.Encoding.PEM, serialization.PrivateFormat.PKCS8,
                                   serialization.NoEncryption()))
    cert_path.write_bytes(cert.public_bytes(serialization.Encoding.PEM))
    if logger:
        logger.info(f"已產生 HMI 自簽憑證：{cert_path}（涵蓋 {', '.join(sorted(dns | ips))}）")
    return str(cert_path), str(key_path)
