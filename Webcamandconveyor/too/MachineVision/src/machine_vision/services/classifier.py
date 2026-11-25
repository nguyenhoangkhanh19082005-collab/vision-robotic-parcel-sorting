from __future__ import annotations

from machine_vision.models import ClassificationResult, Decision, ParcelData
from machine_vision.repositories.shipments import LookupUnavailable, ShipmentRepository


class ParcelClassifier:
    def __init__(self, settings: dict, repository: ShipmentRepository):
        self.settings = settings
        self.repository = repository

    def classify(self, parcel: ParcelData) -> ClassificationResult:
        damage = parcel.damage
        if self.settings.get("require_damage_check", True):
            if damage is None or not damage.calibrated or damage.damaged is None:
                return ClassificationResult(
                    Decision.MANUAL_REVIEW,
                    str(self.settings.get("unknown_bin", "MANUAL_REVIEW")),
                    damage.reason if damage else "Package damage inspection is unavailable",
                    parcel,
                )
            if damage.damaged:
                return ClassificationResult(
                    Decision.REJECT,
                    str(self.settings.get("defect_bin", "DEFECT")),
                    damage.reason,
                    parcel,
                )

        logo = parcel.logo
        if self.settings.get("require_logo", True):
            if logo is None or not logo.calibrated or logo.present is None:
                return ClassificationResult(
                    Decision.MANUAL_REVIEW,
                    str(self.settings.get("unknown_bin", "MANUAL_REVIEW")),
                    logo.reason if logo else "Logo inspection is unavailable",
                    parcel,
                )
            if not logo.present:
                return ClassificationResult(
                    Decision.REJECT,
                    str(self.settings.get("label_reject_bin", "REJECT")),
                    logo.reason,
                    parcel,
                )

        missing = []
        for required, value, label in (
            (self.settings.get("require_qr", True), parcel.qr_text, "QR"),
            (self.settings.get("require_waybill", True), parcel.waybill_code, "waybill"),
        ):
            if required and not value:
                missing.append(label)

        if missing:
            return ClassificationResult(
                Decision.REJECT,
                str(self.settings.get("reject_bin", "REJECT")),
                "Missing required fields: " + ", ".join(missing),
                parcel,
            )

        minimum = float(self.settings.get("min_qr_read_ratio", 0.0))
        if minimum > 0 and parcel.qr_read_ratio is not None and parcel.qr_read_ratio < minimum:
            return ClassificationResult(
                Decision.REJECT,
                str(self.settings.get("reject_bin", "REJECT")),
                f"QR khó đọc (rách hoặc mờ): chỉ đọc được {parcel.qr_read_ratio:.0%} số khung "
                f"(cần >= {minimum:.0%})",
                parcel,
            )

        if self.settings.get("local_test_mode", False):
            return ClassificationResult(
                Decision.PASS,
                str(self.settings.get("local_test_bin", "LOCAL_TEST")),
                "Kiểm tra cục bộ đạt: QR đọc được và các mục kiểm tra đang bật đều hợp lệ; "
                "đã bỏ qua truy vấn Supabase",
                parcel,
            )

        if parcel.waybill_code is None:
            return ClassificationResult(
                Decision.MANUAL_REVIEW,
                str(self.settings.get("unknown_bin", "MANUAL_REVIEW")),
                "No lookup key is available for the station database",
                parcel,
            )
        try:
            record = self.repository.find_by_waybill(parcel.waybill_code)
        except LookupUnavailable as exc:
            # Network/database down: we do not know the order, it is not "unknown".
            return ClassificationResult(
                Decision.MANUAL_REVIEW,
                str(self.settings.get("unknown_bin", "MANUAL_REVIEW")),
                str(exc),
                parcel,
            )
        if record is None:
            return ClassificationResult(
                Decision.REJECT,
                str(self.settings.get("reject_bin", "REJECT")),
                "Waybill not found in station database",
                parcel,
            )

        field = str(self.settings.get("destination_field", "destination_bin"))
        destination = record.get(field)
        if not destination:
            return ClassificationResult(
                Decision.MANUAL_REVIEW,
                str(self.settings.get("unknown_bin", "MANUAL_REVIEW")),
                f"Database record has no '{field}' value",
                parcel,
                record,
            )

        return ClassificationResult(
            Decision.PASS,
            str(destination),
            "Validated against station database",
            parcel,
            record,
        )
